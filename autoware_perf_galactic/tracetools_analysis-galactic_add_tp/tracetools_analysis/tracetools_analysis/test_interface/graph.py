import matplotlib.pyplot as plt

class Graph():
    def __init__(self, path, *paths):
        self.paths = [path] + list(paths)

    def export(self, path):
        pass

class Histogram(Graph):
    def export(self, png_path):
        import os
        os.makedirs(os.path.dirname(png_path), exist_ok=True)

        for path in self.paths:
            stats = path.get_stats()
            latency_width = stats['max'] - stats['min']
            min_binnum = 20
            min_binsize_ns = latency_width/min_binnum * 1.0e6
            binsize_ns = min(min_binsize_ns, 1e6)
            latency_ms, hist = path.hist(binsize_ns).get_xy()
            max_ms = path.get_stats()['max']
            plt.step(latency_ms, hist, where='post', label=f'{path.name}')

        plt.xlabel('Latency [ms]')
        plt.ylabel('Probablility')
        plt.title(self.paths[0].name)
        if len(self.paths) > 1:
            plt.legend(loc='upper right')
        plt.savefig(png_path)
        plt.clf()

class Timeseries(Graph):
    def export(self, png_path):
        import os
        os.makedirs(os.path.dirname(png_path), exist_ok=True)

        for path in self.paths:
            if path.timeseries.clock is None:
                time_ns, latency_ns = path.timeseries.get_xy(use_simtime=False)
            else:
                time_ns, latency_ns = path.timeseries.get_xy(use_simtime=True)
            latency_ms = latency_ns * 1.0e-6
            plt.step(time_ns, latency_ms, label=f'{path.name}', where='post')

        plt.xlabel('Time [s]')
        plt.ylabel('Latency [ms]')
        plt.title(self.paths[0].name)
        if len(self.paths) > 1:
            plt.legend(loc='upper right')
        plt.savefig(png_path)
        plt.clf()
