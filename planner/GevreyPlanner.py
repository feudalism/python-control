from .BasePlanner import Planner

import numpy as np
import math
import scipy as sp
from scipy import special

class GevreyPlanner(Planner):
    """Planner that uses a Gevrey function approach and plans trajectories that are infinitely differentiable.
               /   0                                        t < t0
    phi(t) =  |    1/2(1 + tanh( (2T-1) / (4T(1-T))^s ))    t in [t0, tf]
               \   1                                        t > tf

    T = (t-t0)/(tf-t0)

               /   yA                       t < t0
    y_d(t) =  |    yA + (yB - yA)*phi(t)    t in [t0, tf]
               \   yB                       t > tf

    based on: "J. Rudolph, J. Winkler, F. Woittenek: Flatness Based Control of Distributed Parameter Systems:
    Examples and Computer Exercises from Various Technological Domains" Pages 88ff.
    """


    def __init__(self, YA, YB, t0, tf, d, s):
        super(GevreyPlanner, self).__init__(YA, YB, t0, tf, d)
        self.s = s
        if any(self.YA[1:]!=0) or any(self.YB[1:]!=0):
            print('Boundary conditions of the derivatives set to 0. All given values are ignored.')

    def eval(self, t):
        """Evaluates the planned trajectory at time t.

        Args:
            t (int, float): time

        Returns:
            Y (ndarray): y and its derivatives at t
        """
        Y = np.zeros([(self.d + 1)])
        if t < self.t0:
            Y[0] = self.YA[0]
        elif t > self.tf:
            Y[0] = self.YB[0]
        else:
            T = min(max((t-self.t0)/(self.tf-self.t0),0.001),0.999)
            phi = self.phi(T)
            Y = np.zeros_like(phi)
            Y[0] = self.YA[0] + (self.YB[0] - self.YA[0]) * phi[0]
            for i in range(1, self.d + 1):
                Y[i] = (1 / (self.tf - self.t0)) ** i * (self.YB[0] - self.YA[0]) * phi[i]
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

    def phi(self, t):
        """Calculates phi = 1/2*(1 + tanh( 2(2t-1) / (4t(1-t))^s )) ) and it's derivatives up to order d"""
        phi = np.zeros([self.d + 1])
        phi[0] = 1/2*(1 + self.y(t, 0))
        for i in range(1, self.d + 1):
            phi[i] = 1/2*self.y(t, i)
        return phi


    def y(self, t, n):
        """Calculates y = tanh( 2(2t-1) / (4t(1-t))^s )) and it's derivatives up to order n"""
        s = self.s
        if n == 0:
            # eq. A.3
            y = np.tanh(2*(2*t - 1) / ((4*t*(1 - t))**s))
        elif n == 1:
            # eq. A.5
            y = self.a(t, 2)*(1 - self.y(t, 0)**2)
        else:
            # eq. A.7
            y = sum(sp.special.binom(n - 1, k)*self.a(t, k + 2)*self.z(t, n - 1 - k) for k in range(0, n))
        return y


    def a(self, t, n):
        s = self.s
        if n == 0:
            # eq. A.4
            a = ((4*t*(1 - t))**(1 - s))/(2*(s - 1))
        elif n == 1:
            # eq. for da/dt
            a = 2*(2*t - 1) / ((4*t*(1 - t))**s)
        else:
            # eq. for the n-th derivative of a
            a = 1/(t*(1 - t))*((s - 2+n)*(2*t - 1)*self.a(t, n - 1) + (n - 1)*(2*s - 4 + n)*self.a(t, n - 2))
        return a


    def z(self, t, n):
        if n == 0:
            # eq. A.6
            z = (1-self.y(t, 0)**2)
        else:
            # eq. for n-th derivative of z
            z = - sum(sp.special.binom(n, k)*self.y(t, k)*self.y(t, n - k) for k in range(0, n + 1))
        return z
