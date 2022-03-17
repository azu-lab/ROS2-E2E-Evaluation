import numpy as np
from .util import Util

class Histogram:
    pass

class Histogram:
    __normalize = True

    def __init__(self, raw: np.array, binsize_ns=1):
        binsize_ns = int(binsize_ns)
        self._binsize_ns = binsize_ns
        raw = np.trim_zeros(raw, 'b')
        raw = np.append(raw, 0)
        self._latencies, self._hists = self._divide(raw)

    @classmethod
    def normalize(cls, use):
        cls.__normalize = use

    @classmethod
    def sum(cls, histgrams):
        hist = Histogram(histgrams[0].raw, binsize_ns=histgrams[0].binsize_ns)
        for histgram in histgrams[1:]:
            hist = hist + histgram
        return hist

    def __add__(self, hist: Histogram):
        import itertools
        assert self.binsize_ns == hist.binsize_ns, f'{self.binsize_ns}, {hist.binsize_ns}'
        
        def add_latencies(latency1, latency2):
            latency_min = min(latency1) + min(latency2)
            latency_max = max(latency1) + max(latency2)+1
            return np.arange(latency_min, latency_max + self._binsize_ns, self._binsize_ns)
        
        hist_pairs = list(itertools.product(self._hists, hist._hists))
        latency_pairs = list(itertools.product(self._latencies, hist._latencies))

        hists = [0] * len(hist_pairs)
        latencies = [0] * len(latency_pairs)
        for i, (hist_pair, latency_pair) in enumerate(zip(hist_pairs, latency_pairs)):
            hist = np.convolve(hist_pair[0], hist_pair[1])/2.0
            hist = np.append(hist, 0)
            hist[1:] += hist[:-1]
            hists[i] = hist
            latencies[i] = add_latencies(latency_pair[0], latency_pair[1])
            
        return self.__class__(self._to_raw(latencies, hists), binsize_ns=self._binsize_ns)
    
    @property
    def binsize_ns(self):
        return self._binsize_ns
    
    @property
    def raw(self) -> np.ndarray:
        return self._to_raw(self._latencies, self._hists)
    
    @property
    def latency(self):
        return self._get_latency_ms()
    
    def get_xy(self):
        return self._get_latency_ms(), self._get_hist()
    
    def _divide(self, raw):
        idx = np.where(raw > 0)[0]
        idx_diff = idx[1:] - idx[:-1]

        idx_split = np.where(idx_diff > 1)[0]+1

        indicies = np.array(np.split(idx, idx_split))

        latencies = self._to_latency(indicies)
        hist = np.array([raw[_] for _ in indicies])

        return latencies, hist
    
    def _to_latency(self, index):
        return index * self._binsize_ns
    
    def _to_index(self, latency):
        return int(latency / self._binsize_ns)
    
    def _to_raw(self, latencies, hists):
        max_latency = np.max(Util.flatten(latencies))
        array_size = self._to_index(max_latency)+2
        raw = np.zeros(array_size)
        
        for hist, latency in zip(hists, latencies):
            idx_min = self._to_index(np.min(latency))
            idx_max = self._to_index(np.max(latency))+1
            raw[idx_min:idx_max]+=hist
            
        if Histogram.__normalize:
            sum = np.sum(raw)
            assert(sum != 0)
            return raw / sum
        
        return raw
    
    def _get_latency_ms(self):
        array_size = self._to_index(np.max(Util.flatten(self._latencies))) + 2
        indicies = np.arange(array_size)
        latencies_ns = np.array([self._to_latency(_) for _ in indicies])
        
        offset = max(0, self._to_index(np.min(Util.flatten(self._latencies)))-1)
        return latencies_ns[offset:] * 1.0e-6
    
    def _get_hist(self):
        raw = self._to_raw(self._latencies, self._hists)
        
        min_idx = np.min(np.where(raw>0)[0])
        min_idx = min_idx-1 if min_idx>0 else min_idx
        
        return raw[min_idx:]

class Timeseries:
    def __init__(self, raw: np.array, time: np.array, clock=None):
        self.__raw = raw
        self._time = time
        self._clock = clock

    @classmethod
    def sum(cls, timeseries):
        import pandas as pd

        child_dfs = []

        for child in timeseries:
            child_dfs.append(pd.DataFrame(data={
                'timestamp': child.time,
                'latency': child.raw,
                'clock': child.clock,
                'object': child
            }))

        path_df = pd.concat(child_dfs).sort_values('timestamp')

        first_record_stamp = np.max([df.timestamp[0] for df in child_dfs])
        first_records_df = [df[df['timestamp'] <= first_record_stamp]
                            for df in child_dfs]
        first_record_df = [df.iloc[-1, :] for df in first_records_df]

        target_path_df = path_df[path_df['timestamp'] >= first_record_stamp]
        target_path_df.reset_index(drop=True, inplace=True)

        latency = np.zeros(len(target_path_df))

        child_latencies = np.array([_.latency for _ in first_record_df])

        for i, row in target_path_df.iterrows():
            for j, child in enumerate(timeseries):
                if row['object'] == child:
                    child_latencies[j] = row['latency']
            latency[i] = np.sum(child_latencies)

        timestamp = target_path_df['timestamp'].values
        has_clock = (target_path_df['clock'].values != None).all()
        clock = target_path_df['timestamp'].values if has_clock else None

        return Timeseries(latency, timestamp, clock)

    @property
    def raw(self) -> np.ndarray:
        return self.__raw

    @property
    def raw_nan_removed(self) -> np.ndarray:
        return np.array([_ for _ in self.__raw if not np.isnan(_)])

    @property
    def time(self):
        return self._time

    @property
    def clock(self):
        return self._clock

    def get_xy(self, use_simtime=False):
        if use_simtime:
            assert self.clock is not None, 'Failed to get simtime.'
            return self.clock, self.raw
        return self.time, self.raw

    def to_hist(self, binsize_ns):
        raw = self.raw_nan_removed / binsize_ns
        bins = int(np.ceil(np.max(raw))) + 1
        assert bins < 10000000, 'too large bin size.'
        range_max = int(np.ceil(np.max(raw))) + 1
        hist_raw, _ = np.histogram(raw, bins=bins, range=(0, range_max))
        return Histogram(hist_raw, binsize_ns=binsize_ns)
