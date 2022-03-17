import os
import pandas as pd

class Util():
    @classmethod
    def flatten(cls, x):
        import itertools
        return list(itertools.chain.from_iterable(x))

    @classmethod
    def ext(cls, path):
        import os
        _, ext = os.path.splitext(path)
        return ext[1:]

    @classmethod
    def ns_to_ms(cls, x):
        return x * 1.0e-6

    @classmethod
    def get_ext(cls, path):
        return os.path.basename(path).split('.')[-1]

    @classmethod
    def to_ns_and_name(cls, nodename):
        strs = nodename.split('/')
        ns = '/'.join(strs[:-1])+'/'
        name = strs[-1]
        return ns, name

    @classmethod
    def get_stats_from_hist(cls, hist):
        import numpy as np
        hist = hist(binsize_ns=1000000)
        x, y = hist.get_xy()

        stats = {
            'unit': 'ms',
            'max': max(x),
            'min': min(x),
            'median': x[np.argmax(y)],
            'mean': np.sum(x*y)
        }
        return stats


class DataFrameFilter():

    def __init__(self):
        self.min_limit = None
        self.max_limit = None

    def remove(self, df: pd.DataFrame, target_column_name: str):
        if self.min_limit is not None:
            df = df[df[target_column_name] > self.min_limit]

        if self.max_limit is not None:
            df = df[df[target_column_name] < self.max_limit]

        df.reset_index(inplace=True, drop=True)

        return df

class Counter():
    def __init__(self):
        self._paths = {}
        self._counter = {}

    def _get_key(self, path, base_name):
        if path.child is None:
            return tuple(base_name)

        return tuple(set([tuple(path.child), base_name]))

    def add(self, path, base_name):
        key = self._get_key(path, base_name)

        if path not in self._paths:
            if base_name not in self._counter:
                self._counter[base_name] = 0
            self._paths[key] = self._counter[base_name]
            self._counter[base_name] += 1

    def get_count(self, path, base_name):
        key = self._get_key(path, base_name)
        return self._paths[key]
