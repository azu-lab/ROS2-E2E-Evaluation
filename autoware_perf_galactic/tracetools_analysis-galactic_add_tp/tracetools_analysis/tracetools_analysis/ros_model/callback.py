import collections.abc

from .search_tree import Path
from .publish import Publish

import numpy as np

class CallbackCollectionIterator(collections.abc.Iterator):
    def __init__(self, callback_collection):
        self._i = 0
        self._callback_collection = callback_collection

    def __next__(self):
        try:
            v = self._callback_collection._callbacks[self._i]
            self._i += 1
            return v
        except IndexError:
            raise StopIteration


class CallbackCollection(collections.abc.Iterable):
    def __init__(self):
        self._callbacks = []

    def append(self, callback):
        assert(isinstance(callback, Callback))
        self._callbacks.append(callback)

    def get_from_symbol(self, symbol):
        for callback in self._callbacks:
           if callback.symbol == symbol:
                return callback

        return None

    def __getitem__(self, key):
        return self._callbacks[key]

    def __len__(self):
        return len(self._callbacks)

    def get_timer(self):
        return list(filter(lambda x: isinstance(x, TimerCallback), self._callbacks))

    def get_subscription(self):
        return list(filter(lambda x: isinstance(x, SubscribeCallback), self._callbacks))

    def __iter__(self):
        return CallbackCollectionIterator(self)


class CallbackFactory():
    ignore_topic_name = ['/rosout', '/parameter_events']

    @classmethod
    def create(cls, callback_info, node):
        if callback_info['type'] == 'timer_callback':
            callback = TimerCallback(
                node=node,
                period=callback_info['period'], symbol=callback_info['symbol'])
        elif callback_info['type'] == 'subscribe_callback':
            if callback_info['topic_name'] in CallbackFactory.ignore_topic_name:
                return
            callback = SubscribeCallback(
                node=node,
                topic_name=callback_info['topic_name'], symbol=callback_info['symbol'])

        return callback


class Callback():
    def __init__(self, node, symbol=None):
        self.node = node
        self.symbol = symbol
        self.object = None
        self.path = CallbackPath(self)

    def has_publish(self):
        return self in self.node.pubs.callbacks()

    @property
    def publishes(self):
        return [pub for pub in self.node.pubs if pub.callback == self]

    @property
    def hist(self):
        return self.path.hist

    @property
    def timeseries(self):
        return self.path.timeseries

    @timeseries.setter
    def timeseries(self, timeseries):
        self.path.timeseries = timeseries

    @property
    def name(self):
        return self.path.name

    @property
    def unique_name(self):
        return self.path.unique_name

    def get_info(self):
        pass

    def get_stats(self):
        data = {
            'unit': 'ms',
            'min': np.min(self.timeseries.raw_nan_removed) * 1.0e-6,
            'max': np.max(self.timeseries.raw_nan_removed) * 1.0e-6,
            'median': np.median(self.timeseries.raw_nan_removed) * 1.0e-6,
            'mean': np.mean(self.timeseries.raw_nan_removed * 1.0e-6)
        }
        return data


class CallbackPath(Path):
    def __init__(self, callback):
        super().__init__()
        self.child = []
        self._callback = callback
        self._unique_name = self._callback.symbol

    def is_target(self):
        return self._callback.has_publish()

    @property
    def object(self):
        return self._callback.object

    @property
    def publishes(self):
        return self._callback.publishes

    @property
    def symbol(self):
        return self._callback.symbol

    @property
    def topic_name(self):
        if not isinstance(self._callback, SubscribeCallback):
            return ''
        return self._callback.topic_name

    def get_stats(self):
        data = {
            'unit': 'ms',
            'min': np.min(self.timeseries.raw_nan_removed) * 1.0e-6,
            'max': np.max(self.timeseries.raw_nan_removed) * 1.0e-6,
            'median': np.median(self.timeseries.raw_nan_removed) * 1.0e-6,
            'mean': np.mean(self.timeseries.raw_nan_removed) * 1.0e-6,
            'send': len(self.timeseries.raw),
            'lost': len(self.timeseries.raw)-len(self.timeseries.raw_nan_removed)
        }
        return data


class TimerCallback(Callback):
    def __init__(self, node, period=None, symbol=None):
        super().__init__(node=node, symbol=symbol)
        self.period = period

    def get_info(self):
        info = {
            'type': 'timer_callback',
            'period': self.period,
            'symbol': self.symbol,
        }
        return info


class SubscribeCallback(Callback):
    def __init__(self, node, topic_name=None, symbol=None):
        super().__init__(node=node, symbol=symbol)
        self.topic_name = topic_name

    def get_info(self):
        info = {
            'type': 'subscribe_callback',
            'topic_name': self.topic_name,
            'symbol': self.symbol,
        }
        return info
