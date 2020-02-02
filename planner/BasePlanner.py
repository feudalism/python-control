from abc import ABC, abstractmethod

class Planner(ABC):
    """ Base class for a trajectory planner.

    Attributes:
        YA (int, float, ndarray): start value (size = d+1)
        YB (int, float, ndarray): final value (size = d+1)
        t0 (int, float): start time
        tf (int, float): final time
        d (int): trajectory is smooth up at least to the d-th derivative
    """

    def __init__(self, YA, YB, t0, tf, d):
        self.YA = YA
        self.YB = YB
        self.t0 = t0
        self.tf = tf
        self.d = d

    @abstractmethod
    def eval(self):
        return