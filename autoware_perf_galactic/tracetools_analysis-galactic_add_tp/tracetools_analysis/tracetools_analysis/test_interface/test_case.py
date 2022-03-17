import math
import numpy as np

from .common import Result

class TestTarget():
    def __init__(self, unit, path):
        self.path = path
        self.unit = unit
        self.tests = []

    def add_test(self, test_case):
        assert(isinstance(test_case, TestCase))
        self.tests.append(test_case)

    def judge(self):
        results = [test.judge() for test in self.tests]

        if Result.NOT_ACCEPTABLE in results:
            return Result.NOT_ACCEPTABLE

        if Result.ACCEPTABLE in results:
            return Result.ACCEPTABLE

        return Result.DESIRED

class TestCaseFactory():
    def __init__(self, unit, target):
        self.unit = unit
        self.target = target

    def create(self, test_case, desirable=None, acceptable=None):
        if test_case == 'mean':
            return MeanTest(self.unit, self.target, desirable, acceptable)
        elif test_case == 'peak':
            return PeakTest(self.unit, self.target, desirable, acceptable)
        elif test_case == 'std':
            return StdTest(self.unit, self.target, desirable, acceptable)
        print('failed to parse test case: {}'.format(test_case))

class TestCase():
    def __init__(self, unit, target, desirable=None, acceptable=None):
        self.unit = unit
        self.target = target
        self.desirable = desirable
        self.acceptable = acceptable
        
        if desirable is not None and acceptable is not None:
            assert desirable <= acceptable, \
                "{} {} value error. desirable > acceptable.".format( \
                    target.path.name, self.__class__.__name__)
            
    def has_desirable_and_acceptable(self):
        return self.desirable is not None and self.acceptable
    
    def judge(self) -> Result:
        if not self.has_desirable_and_acceptable():
            return None
        
        stat = self.get_stat()
        if stat <= self.desirable:
            return Result.DESIRED
        
        elif stat <= self.acceptable:
            return Result.ACCEPTABLE
        else:
            return Result.NOT_ACCEPTABLE
        
    def get_stat(self) -> float:
        return 0.0

class PeakTest(TestCase):
    def get_stat(self) -> float:
        return self.target.path.get_stats()['max']
    
    def __str__(self):
        return 'peak'

class MeanTest(TestCase):
    def get_stat(self) -> float:
        return self.target.path.get_stats()['mean']
    
    def __str__(self):
        return 'mean'

class StdTest(TestCase):
    def get_stat(self) -> float:
        latency_ms, p = self.target.path.hist(binsize_ns=1e6).get_xy()
        mean = self.target.path.get_stats()['mean']
        std_dev = math.sqrt(np.sum((latency_ms - mean)**2 * p))
        return std_dev
    
    def __str__(self):
        return 'std'
