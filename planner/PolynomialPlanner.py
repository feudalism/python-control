from .BasePlanner import Planner

import numpy as np
import math

class PolynomialPlanner(Planner):
    """Planner subclass that uses a polynomial approach for trajectory generation

    Attributes:
        c (ndarray): parameter vector of polynomial

    """

    def __init__(self, YA, YB, t0, tf, d):
        super().__init__(YA, YB, t0, tf, d)
        self.c = self.coefficients()

    def eval(self, t):
        """Evaluates the planned trajectory at time t.

        Args:
            t (int, float): time

        Returns:
            Y (ndarray): y and its derivatives at t
        """
        if t < self.t0:
            Y = self.YA
        elif t > self.tf:
            Y = self.YB
        else:
            Y = np.dot(self.TMatrix(t), self.c)
        return Y

    def eval_vec(self, tt):
        """Samples the planned trajectory

        Args:
            tt (ndarray): time vector

        Returns:
            Y (ndarray): y and its derivatives at the sample points

        """
        Y = np.zeros([len(tt), len(self.YA)])
        for i in range(0, len(tt)):
            Y[i] = self.eval(tt[i])
        return Y
		
    def TMatrix(self, t):
        """Computes the T matrix at time t

        Args:
            t (int, float): time

        Returns:
            T (ndarray): T matrix

        """

        d = self.d
        n = d+1 # first dimension of T
        m = 2*d+2 # second dimension of T

        T = np.zeros([n, m])

        for i in range(0, m):
            T[0, i] = t ** i / math.factorial(i)
        for j in range(1, n):
            T[j, j:m] = T[0, 0:m-j]
        return T
		
    def coefficients(self):
        """Calculation of the polynomial parameter vector

        Returns:
            c (ndarray): parameter vector of the polynomial

        """
        t0 = self.t0
        tf = self.tf

        Y = np.append(self.YA, self.YB)

        T0 = self.TMatrix(t0)
        Tf = self.TMatrix(tf)

        T = np.append(T0, Tf, axis=0)

        # solve the linear equation system for c
        c = np.linalg.solve(T, Y)
        return c
