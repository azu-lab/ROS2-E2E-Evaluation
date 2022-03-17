import collections.abc
import numpy as np

from .callback import Callback, CallbackPath
from .search_tree import Path

class SchedCollectionIterator(collections.abc.Iterator):
    def __init__(self, sched_collection):
        self._i = 0
        self._sched_collection = sched_collection

    def __next__(self):
        try:
            v = self._sched_collection._scheds[self._i]
            self._i += 1
            return v
        except IndexError:
            raise StopIteration


class SchedCollection(collections.abc.Iterable):
    def __init__(self):
        self._scheds = []

    def append(self, sched):
        assert(isinstance(sched, Sched)), sched
        self._scheds.append(sched)

    def get(self, callback_in, callback_out):
        def f(x):
            return x.callback_in == callback_in and x.callback_out == callback_out

        scheds = list(filter(lambda x: f(x), self._scheds))
        return scheds[0]

    def get_info(self):
        callback_dependency = {}
        for sched in self._scheds:
            callback_dependency[sched.callback_in.symbol] = sched.callback_out.symbol
        if len(callback_dependency) == 0:
            callback_dependency[''] = ''
        return callback_dependency

    def has(self, callback_in, callback_out):
        for sched in self._scheds:
            if sched.callback_in == callback_in and sched.callback_out == callback_out:
                return True
        return False

    def get(self, callback_in, callback_out):
        assert isinstance(callback_in, Callback) or isinstance(callback_in, CallbackPath)
        assert isinstance(callback_out, Callback) or isinstance(callback_in, CallbackPath)

        for sched in self._scheds:
            if sched.callback_in == callback_in and sched.callback_out == callback_out:
                return sched

        raise KeyError('failed to find sched object. callback_in:{} callback_out:{}'.format(callback_in.name, callback_out.name))

    def __iter__(self):
        return SchedCollectionIterator(self)

    def __len__(self):
        return len(self._scheds)

    def __getitem__(self, key):
        return self._scheds[key]


class Sched(Path):
    def __init__(self, callback_in, callback_out):
        assert isinstance(callback_in, Callback), callback_in
        assert isinstance(callback_out, Callback), callback_out

        super().__init__()
        self.callback_in = callback_in
        self.callback_out = callback_out
        self._unique_name = '{}--{}'.format(self.callback_in.name, self.callback_out.name)

    def get_stats(self):
        data = {
            'unit': 'ms',
            'min': np.min(self.timeseries.raw_nan_removed)*1.0e-6,
            'max': np.max(self.timeseries.raw_nan_removed)*1.0e-6,
            'median': np.median(self.timeseries.raw_nan_removed)*1.0e-6,
            'mean': np.mean(self.timeseries.raw_nan_removed)*1.0e-6,
            'send': len(self.timeseries.raw),
            'lost': len(self.timeseries.raw)-len(self.timeseries.raw_nan_removed)
        }
        return data
