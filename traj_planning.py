# # Trajectory planning - polynomial

import numpy as np
from planner import PolynomialPlanner, PrototypePlanner, GevreyPlanner
import matplotlib.pyplot as plt

# Start position
YA = np.array([0, 0, 0])
t0 = 0

# End position
YB = np.array([1, 0, 0])
tf = 1

# Smoothness
d = 2

# Gevrey parameters
s1 = 1.1
s2 = 1.9

# Generate trajectory objects
y_poly = PolynomialPlanner(YA, YB, t0, tf, d)
y_prot = PrototypePlanner(YA, YB, t0, tf, d)
y_gev1 = GevreyPlanner(YA, YB, t0, tf, d, s1)
y_gev2 = GevreyPlanner(YA, YB, t0, tf, d, s2)

# Sample the generated trajectory
tt = np.linspace(t0, tf, 100)
Y_poly = y_poly.eval_vec(tt)
Y_prot = y_prot.eval_vec(tt)
Y_gev1 = y_gev1.eval_vec(tt)
Y_gev2 = y_gev2.eval_vec(tt)

def plot_finalise():
	plt.legend([r'$y_d(t)$', r'$\dot{y}_d(t)$',r'$\ddot{y}_d(t)$'])
	plt.xlabel(r't in s')
	plt.grid(True)

def plot_data(Y_vec, title):
	plt.figure()
	plt.plot(tt, Y_vec)
	plt.title(title)
	plot_finalise()

plot_data(Y_poly, 'Polynomial')
plot_data(Y_prot, 'Prototype')
plot_data(Y_gev1, 'Gevrey s={}'.format(s1))
plot_data(Y_gev2, 'Gevrey s={}'.format(s2))

plt.show()