from .BasePlanner import Planner

import numpy as np
import math
import scipy as sp
from scipy import special

class PrototypePlanner(Planner):
    """Planner subclass that uses a polynomial approach for trajectory generation"""

    def __init__(self, YA, YB, t0, tf, d):
        super().__init__(YA, YB, t0, tf, d)
		
        # check if values are 0
        if any(self.YA[1:]!=0) or any(self.YB[1:]!=0):
            print('Boundary conditions of the derivatives set to 0. All given values are ignored.')

    def eval(self, t):
        """Evaluates the planned trajectory at time t.

                Args:
                    t (int, float): time

                Returns:
                    Y (ndarray): y and its derivatives at t
                """

        phi = self.prototype_fct((t - self.t0) / (self.tf - self.t0))
        Y = np.zeros([(self.d + 1)])
        if t < self.t0:
            Y[0] = self.YA[0]
        elif t > self.tf:
            Y[0] = self.YB[0]
        else:
            Y[0] = self.YA[0] + (self.YB[0]-self.YA[0])*phi[0]
            for i in range(1,self.d+1):
                Y[i] = (1/(self.tf-self.t0))**i * (self.YB[0]-self.YA[0])*phi[i]
        return Y


    def eval_vec(self,tt):
        """Samples the planned trajectory

        Args:
            tt (ndarray): time vector

        Returns:
            Y (ndarray): y and its derivatives at the sample points

        """
        Y = np.zeros([len(tt),len(self.YA)])
        for i in range(0,len(tt)):
            Y[i] = self.eval(tt[i])
        return Y


    def prototype_fct(self, t):
        """Prototype function, that is used in the path planner and returns a polynomial and its derivatives up to order gamma.

        Args:
            t: time
        Returns: phi (vector of phi and its successive derivatives)

        """
        phi = np.zeros([self.d + 1])

        summation = sum([special.binom(self.d, k) * (-1) ** k * t ** (k + self.d + 1) / (self.d + k + 1)
                         for k in range(0, self.d + 1)])
        phi[0] = self.faculty(2 * self.d + 1) / (self.faculty(self.d) ** 2) * summation

        # calculate it's derivatives up to order (d-1)

        for p in range(1, self.d + 1):
            summation = sum(
                [sp.special.binom(self.d, k) * (-1) ** k * t ** (k + self.d + 1 - p) / (self.d + k + 1) * self.prod_iter(k, p)
                 for k in range(0, self.d + 1)])
            phi[p] = self.faculty(2 * self.d + 1) / (self.faculty(self.d) ** 2) * summation

        return phi


    def faculty(self, x):
        """Calcualtes the faculty of x"""
        result = 1
        for i in range(2, x + 1):
            result *= i
        return result


    def prod_iter(self, k, p):
        """Calculates the iterative product"""
        result = 1
        for i in range(1, p + 1):
            result *= (self.d + k + 2 - i)
        return result