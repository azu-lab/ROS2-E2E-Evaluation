import collections.abc

from .search_tree import Path
from .node import NodePath, Node
from .util import Counter
from .callback import SubscribeCallback
from .publish import Publish

import numpy as np

class DDS(Path):
    counter = Counter()

    def __init__(self, publish, node_pub, node_sub):
        self.child = []
        super().__init__(self.child)
        assert(isinstance(node_pub, Node))
        assert(isinstance(node_sub, Node))
        self.node_pub = node_pub
        self.node_sub = node_sub
        self.publish = publish
        self.topic_name = publish.topic_name

        self.counter.add(self, self.topic_name)
        self._index = self.counter.get_count(self, self.topic_name)
        self._unique_name = '{}_dds_{}'.format(self.topic_name, self._index)

    def get_stats(self):
        data = {
            'unit': 'ms',
            'min': None,
            'max': None,
            'median': None,
            'mean': None,
            'send': None,
            'lost': None
        }

        if len(self.timeseries.raw_nan_removed) == 0:
            return data

        data['min'] = np.min(self.timeseries.raw_nan_removed) * 1.0e-6
        data['max'] = np.max(self.timeseries.raw_nan_removed) * 1.0e-6
        data['median'] = np.median(self.timeseries.raw_nan_removed) * 1.0e-6
        data['mean'] = np.mean(self.timeseries.raw_nan_removed) * 1.0e-6
        data['send'] = len(self.timeseries.raw)
        data['lost'] = len(self.timeseries.raw)-len(self.timeseries.raw_nan_removed)

        return data


class Comm(Path):
    counter = Counter()

    def __init__(self, publish, node_pub, node_sub, cb_pub, cb_sub):
        assert(isinstance(node_pub, Node))
        assert(isinstance(node_sub, Node))
        assert(isinstance(publish, Publish))

        self.child = [DDS(publish, node_pub, node_sub)]
        super().__init__(self.child)

        self.topic_name = publish.topic_name
        self.node_pub = node_pub
        self.node_sub = node_sub
        self.cb_pub = cb_pub
        self.cb_sub = cb_sub

        self.counter.add(self, self.topic_name)
        self._index = self.counter.get_count(self, self.topic_name)
        self.publish = publish
        self._unique_name = '{}_{}'.format(self.topic_name, self._index)

    def get_objects(self):
        return {'publish': self.publish.object, 'subscribe': self.cb_sub.object}

    def get_stats(self):
        data = {
            'unit': 'ms',
            'min': None,
            'max': None,
            'median': None,
            'mean': None,
            'send': None,
            'lost': None
        }

        if len(self.timeseries.raw_nan_removed) == 0:
            return data

        data['min'] = np.min(self.timeseries.raw_nan_removed) * 1.0e-6
        data['max'] = np.max(self.timeseries.raw_nan_removed) * 1.0e-6
        data['median'] = np.median(self.timeseries.raw_nan_removed) * 1.0e-6
        data['mean'] = np.mean(self.timeseries.raw_nan_removed) * 1.0e-6
        data['send'] = len(self.timeseries.raw)
        data['lost'] = len(self.timeseries.raw)-len(self.timeseries.raw_nan_removed)

        return data


class CommCollectionIterator(collections.abc.Iterator):
    def __init__(self, inter_node_collection):
        self._i = 0
        self._inter_node_collection = inter_node_collection

    def __next__(self):
        try:
            v = self._inter_node_collection._comms[self._i]
            self._i += 1
            return v
        except IndexError:
            raise StopIteration


class CommCollection(collections.abc.Iterable):
    def __init__(self):
        self._comms = []

    def append(self, inter_node):
        self._comms.append(inter_node)

    def __iter__(self):
        return CommCollectionIterator(self)

    def __getitem__(self, key):
        return self._comms[key]

    def __len__(self):
        return len(self._comms)

    def has(self, node_pub, node_sub):
        assert isinstance(node_pub, NodePath)
        assert isinstance(node_sub, NodePath)

        return self.get(node_pub, node_sub) is not None

    def get(self, node_pub, node_sub):
        assert isinstance(node_pub, NodePath)
        assert isinstance(node_sub, NodePath)

        topic_name = node_sub.child[0].topic_name
        for comm in self._comms:
            if comm.node_pub == node_pub.node and \
               comm.node_sub == node_sub.node and \
               comm.topic_name == topic_name:
                return comm
        return None

    def get_unlinked(self):
        return [comm for comm in self._comms if comm.cb_pub is None]
