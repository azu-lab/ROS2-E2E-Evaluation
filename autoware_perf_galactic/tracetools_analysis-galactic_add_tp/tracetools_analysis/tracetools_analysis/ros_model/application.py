from tracetools_analysis.utils.ros2 import Ros2DataModelUtil
from tracetools_analysis.loading import load_file
from tracetools_analysis.processor.ros2 import Ros2Handler

from .util import Util, DataFrameFilter, Counter
from .node import Node, NodePath, NodeFactory, NodeCollection
from .comm import CommCollection, Comm
from .callback import SubscribeCallback, TimerCallback, Callback
from .publish import Publish
from .search_tree import SearchTree, Path, PathCollection
from .data_type import Timeseries

import pandas as pd
import numpy as np


class End2End(Path):
    counter = Counter()
    base_name = 'end_to_end'

    def __init__(self, child):
        super().__init__(child)
        self.counter.add(self, self.base_name)
        self._index = self.counter.get_count(self, self.base_name)
        self._unique_name = '{}_{}'.format(self.base_name, self._index)

    def _get_node_paths(self):
        return list(filter(lambda x: isinstance(x, NodePath), self.child))

    @property
    def child_names(self):
        return '--'.join([_.name for _ in self._get_node_paths()])

    def get_stats(self):
        return Util.get_stats_from_hist(self.hist)

class TimeConverter():
    def __init__(self, events):
        import numpy as np
        x = [event['_timestamp'] for event in events]
        y = [event['stamp'] for event in events]
        a, b = np.polyfit(x, y, 1)
        self._f = lambda x: a*x + b

    def to_clock(self, time: np.ndarray):
        return np.array([self._f(t) for t in time])

class Application():
    def __init__(self):
        self.nodes = NodeCollection()
        self.data_util = None
        self.__paths = []
        self.events = None
        self.clocks = None
        self.comms = CommCollection()
        self.comm_instances = None
        self._filter = DataFrameFilter()
        self._time_converter = None

    @property
    def paths(self):
        return self.__paths

    @property
    def scheds(self):
        return Util.flatten([_.scheds for _ in self.nodes])

    @property
    def callbacks(self):
        return Util.flatten([list(node.callbacks) for node in self.nodes])

    def has_start_node(self):
        for node in self.nodes:
            if node.start_node:
                return True
        return False

    def has_end_node(self):
        for node in self.nodes:
            if node.end_node:
                return True
        return False

    def update_paths(self):
        self.__paths = self._search_paths(self.nodes)

    def get_info(self):
        start_node = self.nodes.get_start_node()
        end_node = self.nodes.get_end_node()

        start_node_name = ''
        end_node_name = ''

        if start_node is not None:
            start_node_name = f'{start_node.ns}{start_node.name}'
        if end_node is not None:
            end_node_name = f'{end_node.ns}{end_node.name}'

        path_name_alias = {}
        for path in self.get_path_list():
            if path.unique_name != path.name:
                path_name_alias[path.unique_name] = path.name
        if len(path_name_alias) == 0:
            path_name_alias[''] = ''

        info = {
            'target_path': {
                'start_node_name': start_node_name,
                'end_node_name': end_node_name,
            },
            'nodes': [node.get_info() for node in self.nodes],
            'path_name_alias': path_name_alias
        }
        return info

    def import_trace(self, trace_dir, start_transition_ms=0, end_transition_ms=0, clock=None):
        assert(self.nodes != [])

        events = load_file(trace_dir)

        if clock is not None:
            clocks = load_file(clock)
            self.clocks = clocks
            self._time_converter = TimeConverter(clocks)

        self.events = events

        handler = Ros2Handler.process(events)

        self.data_util = Ros2DataModelUtil(handler.data)
        self._insert_runtime_data(self.data_util, self.nodes)

        self._filter.min_limit = events[0]['_timestamp']+start_transition_ms*1.0e6
        self._filter.max_limit = events[-1]['_timestamp']-end_transition_ms*1.0e6

        callback_durations = handler.data.callback_instances
        if len(callback_durations) > 0:
            callback_durations = self._filter.remove(callback_durations, 'timestamp')
            assert len(callback_durations) > 0, 'all instance are removed'
        self._import_callback_durations(callback_durations)

        sched_instances = self._get_sched_instances(events, self.scheds)
        if len(sched_instances) > 0:
            sched_instances = self._filter.remove(sched_instances, 'timestamp')
            assert len(sched_instances) > 0, 'all instance are removed'
        self._import_sched_durations(sched_instances)

        comm_instances = self._get_comm_instances(events, self.comms)
        if len(comm_instances) > 0:
            comm_instances = self._filter.remove(comm_instances, 'timestamp')
            assert len(comm_instances) > 0, 'all instance are removed'
        self.comm_instances = comm_instances
        self._import_comm_instances(comm_instances)

    def export(self, path):
        import json
        with open(path, mode='w') as f:
            f.write(json.dumps(self.get_info(), indent=2))

    def get_path_list(self):
        targets = []
        def get_child_path(paths):
            for path in paths:
                if path not in targets:
                    targets.append(path)
                get_child_path(path.child)

        get_child_path(self.paths)
        return targets

    def find_path(self, name):
        paths = self.get_path_list()
        target_paths = []
        for path in paths:
            if path.name == name or path.unique_name == name:
                target_paths.append(path)

        assert len(target_paths) > 0, f'Failed to find target path :{name}'
        assert len(target_paths) <= 1, f'Duplicated path names : {name}'

        return target_paths[0]

    def _search_paths(self, nodes):
        # search all path
        paths = PathCollection()
        for node_path in self.nodes.get_root_paths():
            for child in SearchTree.search(node_path):
                if not paths.has(child):
                    paths.append(End2End(child))

        return paths

    def _import_callback_durations(self, callback_instances):
        for node in self.nodes:
            for callback in node.callbacks:
                callback_duration_records = callback_instances[
                    callback_instances['callback_object'] == callback.object]
                callback_durations = callback_duration_records['duration'].values
                time = callback_duration_records['timestamp'].values
                clock = None if self._time_converter is None \
                    else self._time_converter.to_clock(time)
                callback.timeseries = Timeseries(callback_durations, time, clock)

    def _import_comm_instances(self, instances):
        for comm in self.comms:
            objects = comm.get_objects()
            duration_records = instances[
                (instances['publish_object'] == objects['publish']) &
                (instances['subscribe_object'] == objects['subscribe'])]

            if len(duration_records) == 0:
                print(f'Failed to calculate {comm.topic_name} latency.'
                      f'Please confirm {comm.topic_name} contains header msg.')

            time = duration_records['timestamp'].values
            clock = None if self._time_converter is None \
                else self._time_converter.to_clock(time)
            comm.timeseries = Timeseries(duration_records['duration'].values, time, clock)
            dds = comm.child[0]
            dds.timeseries = Timeseries(duration_records['communication_latency'].values, time, clock)

    def get_publish_instances(self, events):
        publish_instances = pd.DataFrame(columns=[
            'timestamp',
            'stamp',
            'publisher_handle'])

        for event in events:
            if event['_name'] == 'ros2:rclcpp_publish':
                data = {
                    'timestamp': event['_timestamp'],
                    'publisher_handle': event['publisher_handle']
                }
                publish_instances = publish_instances.append(data, ignore_index=True)

        publish_instances = pd.merge(publish_instances,  self.data_util.get_publish_info() , on='publisher_handle')
        publish_instances.reset_index(inplace=True, drop=True)
        assert len(publish_instances)>0,'ros2:rclcpp_publish is not recorded. Use forked-foxy.'

        return publish_instances

    def get_subscribe_instances(self, events):
        subscribe_instances = pd.DataFrame(columns=[
            'timestamp',
            'stamp',
            'callback_object',
            'source_stamp',
            'received_stamp'])

        for event in events:
            if event['_name'] == 'ros2:rclcpp_subscribe':
                data = {
                    'timestamp': event['_timestamp'],
                    'callback_object': event['callback'],
                    'source_stamp': event['source_stamp'],
                    'received_stamp': event['received_stamp']
                }
                subscribe_instances = subscribe_instances.append(data, ignore_index=True)

        subscribe_instances = pd.merge(subscribe_instances,  self.data_util.get_subscribe_info(), on='callback_object')
        subscribe_instances.reset_index(inplace=True, drop=True)

        return subscribe_instances

    def _get_comm_instances(self, events, comms):
        publish_instances = self.get_publish_instances(events)
        subscribe_instances = self.get_subscribe_instances(events)

        comm_instances = [self._get_specific_comm_instances(comm,
                                                  publish_instances,
                                                  subscribe_instances) \
                          for comm in comms]
        if len(comm_instances) > 0:
            comm_instances = pd.concat(comm_instances)
        return comm_instances

    def _get_specific_comm_instances(self, comm, publish_df, subscribe_df):
        assert isinstance(publish_df, pd.DataFrame)
        assert isinstance(subscribe_df, pd.DataFrame)
        assert len(publish_df) > 0
        assert len(subscribe_df) > 0

        obj = comm.get_objects()
        publish_object = obj['publish']
        subscribe_object = obj['subscribe']

        comm_instances = pd.DataFrame(columns=['publish_object',
                                               'subscribe_object',
                                               'timestamp',
                                               'duration'])

        # filter specific records
        publish_df_ = publish_df[publish_df['publisher_handle'] == publish_object]
        publish_df_.reset_index(inplace=True, drop=True)

        subscribe_df_ = subscribe_df[subscribe_df['callback_object'] == subscribe_object]
        subscribe_df_.reset_index(inplace=True, drop=True)

        if len(publish_df_) == 0 or len(subscribe_df_) == 0:
            return comm_instances

        for i, subscribe_record in subscribe_df_.iterrows():

            duration_ns = None
            communication_latency_ns = None

            duration_ns = subscribe_record['timestamp'] - subscribe_record['source_stamp']
            communication_latency_ns = subscribe_record['received_stamp'] - subscribe_record['source_stamp']

            data = {
                'timestamp': subscribe_record['timestamp'],
                'publish_object': publish_object,
                'subscribe_object': subscribe_object,
                'duration': duration_ns,
                'communication_latency': communication_latency_ns
            }
            comm_instances = comm_instances.append(data, ignore_index=True)

        # remove last records if values are NaN

        valid_messages_df = comm_instances[comm_instances['duration'] != None]
        last_valid_idx = valid_messages_df.index[-1]
        comm_instances = comm_instances.iloc[:last_valid_idx+1]

        return comm_instances

    def _get_specific_sched_instances(self, sched, callback_end_instances, callback_start_instances):
        sched_instances = pd.DataFrame(columns=[
            'timestamp',
            'callback_in_object',
            'callback_out_object',
            'duration'])

        start_instances = callback_start_instances[
            callback_start_instances['callback_out_object'] == sched.callback_out.object]
        end_instances = callback_end_instances[
            callback_end_instances['callback_in_object'] == sched.callback_in.object]

        start_instances = start_instances.assign(instance_type='start')
        end_instances = end_instances.assign(instance_type='end')

        start_instances.drop('callback_out_object', axis=1)
        end_instances.drop('callback_in_object', axis=1, inplace=True)

        assert len(end_instances) > 0
        assert len(start_instances) > 0

        instances = pd.concat([start_instances, end_instances]).sort_values('timestamp')
        instances.reset_index(drop=True, inplace=True)

        instances_list = list(instances.itertuples())
        for row_, row in zip(instances_list[:-1], instances_list[1:]):
            if row_.instance_type == 'end' and row.instance_type == 'start':
                duration = row.timestamp - row_.timestamp
                data = {
                    'timestamp': row_.timestamp,
                    'callback_in_object': sched.callback_in.object,
                    'callback_out_object': sched.callback_out.object,
                    'duration': duration
                }
                sched_instances = sched_instances.append(data, ignore_index=True)
        return sched_instances

    def _get_callback_end_instances(self, events):
        callback_end_instances = pd.DataFrame(columns=[
            'timestamp',
            'callback_in_object'])

        for event in events:
            if event['_name'] == 'ros2:callback_end':
                data = {
                    'timestamp': event['_timestamp'],
                    'callback_in_object': event['callback']
                }
                callback_end_instances = callback_end_instances.append(data, ignore_index=True)

        return callback_end_instances

    def _get_callback_start_instances(self, events):
        callback_start_instances = pd.DataFrame(columns=[
            'timestamp',
            'callback_out_object'])

        for event in events:
            if event['_name'] == 'ros2:callback_start':
                data = {
                    'timestamp': event['_timestamp'],
                    'callback_out_object': event['callback']
                }
                callback_start_instances = callback_start_instances.append(data, ignore_index=True)

        return callback_start_instances

    def _get_sched_instances(self, events, scheds):
        callback_end_instances = self._get_callback_end_instances(events)
        callback_start_instances = self._get_callback_start_instances(events)

        sched_instances = [
            self._get_specific_sched_instances(sched, callback_end_instances, callback_start_instances)
            for sched in scheds]

        if len(sched_instances) > 0:
          sched_instances = pd.concat(sched_instances)

        return sched_instances


    def _import_sched_durations(self, sched_instances):
        for sched in self.scheds:
            duration_records = sched_instances[
                (sched_instances['callback_in_object'] == sched.callback_in.object) &
                (sched_instances['callback_out_object']
                 == sched.callback_out.object)
            ]
            duration_raw = duration_records['duration'].values
            time = duration_records['timestamp'].values
            clock = None if self._time_converter is None \
                else self._time_converter.to_clock(time)
            sched.timeseries = Timeseries(duration_raw, time, clock)

    def _insert_runtime_data(self, data_util, nodes):
        self._insert_callback_object(data_util)
        self._insert_publish_object(data_util, nodes)

    def _insert_publish_object(self, data_util, nodes):
        for node in nodes:
            for pub in node.publishes:
                pub.object = data_util.get_publish_object(
                    namespace=node.ns, node_name=node.name, topic_name=pub.topic_name)

    def _insert_callback_object(self, data_util):
        # トレース結果のcallback objectを挿入
        for node in self.nodes:
            for callback in node.callbacks:
                if not isinstance(callback, Callback):
                    continue
                callback.object = data_util.get_callback_object(callback.symbol)
                assert callback.object is not None, 'failed to get callback_object'


class ApplicationFactory():

    @classmethod
    def create_from_json(cls, path):
        import itertools
        import json

        app = Application()

        with open(path, 'r') as f:
            app_info = json.load(f)

        for node_info in app_info['nodes']:
            app.nodes.append(NodeFactory.create(node_info))

        for node_pub, node_sub in itertools.product(app.nodes, app.nodes):
            for pub, sub in itertools.product(node_pub.pubs, node_sub.subs):
                if pub.topic_name == sub.topic_name:
                    comm = Comm(pub, node_pub, node_sub,
                                cb_pub=pub.callback, cb_sub=sub)
                    app.comms.append(comm)

        ns, name = Util.to_ns_and_name(app_info['target_path']['start_node_name'])
        node = app.nodes.get_node(ns, name)
        if node is not None:
            node.start_node = True

        ns, name = Util.to_ns_and_name(app_info['target_path']['end_node_name'])
        node = app.nodes.get_node(ns, name)
        if node is not None:
            node.end_node = True

        # find subsequent comms
        node_paths = Util.flatten([node.paths for node in app.nodes])
        for node_path, comm in itertools.product(node_paths, app.comms):
            if comm.cb_pub is None:
                continue
            # TODO: clean
            if node_path.child[-1].symbol == comm.cb_pub.symbol:
                node_path.subsequent.append(comm)
            if node_path.child[0].symbol == comm.cb_sub.symbol:
                comm.subsequent.append(node_path)

        app.update_paths()

        from collections import Counter
        alias_count = Counter(app_info['path_name_alias'].values())
        for alias_name, count in alias_count.items():
            assert count < 2, f'architecture file has duplicate alias name: {alias_name}'

        for path_name, alias_name in app_info['path_name_alias'].items():
            if path_name == '':
                continue
            path = app.find_path(path_name)
            path.alias_name = alias_name

        return app


    @classmethod
    def _get_duplicate_num_max(cls, array):
        count_max = 0
        for element in list(set(array)):
            count_max = max(count_max, len(array[array == element]))
        return count_max

    @classmethod
    def _create_node(cls, node_info, data_util):
        timer_df = data_util.get_timer_info()
        sub_df = data_util.get_subscribe_info()
        pub_df = data_util.get_publish_info()

        ns, name = Util.to_ns_and_name(node_info['name'])
        node = Node(ns=ns, name=name)

        timer_df_ = timer_df[timer_df['name'] == node.name]
        assert ApplicationFactory._get_duplicate_num_max(timer_df_['period'].values) <= 1,\
            '{} node has same period'.format(node.name)
        for i, (_, df) in enumerate(timer_df_.iterrows()):
            callback = TimerCallback(node=node, period=df['period'], symbol=df['symbol'])
            node.callbacks.append(callback)

        sub_df_ = sub_df[sub_df['name'] == node.name]
        assert ApplicationFactory._get_duplicate_num_max(sub_df_['topic_name'].values) <= 1, \
            '{} node has same topic_name'.format(node.name)

        for i, (_, df) in enumerate(sub_df_.iterrows()):
            if df['topic_name'] in ['/parameter_events']:
                continue
            callback = SubscribeCallback(node=node,
                topic_name=df['topic_name'], symbol=df['symbol'])
            node.callbacks.append(callback)

        pub_df_ = pub_df[pub_df['name'] == node.name]
        for _, df in pub_df_.iterrows():
            if df['topic_name'] in ['/rosout', '/parameter_events']:
                continue
            if len(node.callbacks) == 1:
                node.pubs.append(
                    Publish(topic_name=df['topic_name'], callback=node.callbacks[0]))
            else:
                node.pubs.append(
                    Publish(topic_name=df['topic_name'], callback=None))
        return node

    @classmethod
    def _create_app(cls, data_util):
        app = Application()
        for _, node_info in data_util.data.nodes.drop('timestamp', axis=1).iterrows():
            node = ApplicationFactory._create_node(node_info, data_util)
            app.nodes.append(node)
        return app

    @classmethod
    def create_from_trace(cls, path):
        events = load_file(path)

        handler = Ros2Handler.process(events)
        data_util = Ros2DataModelUtil(handler.data)
        app = ApplicationFactory._create_app(data_util)

        return app
