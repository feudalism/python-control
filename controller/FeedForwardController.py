#! /usr/bin/python3

# FeedForwardController.py

import numpy as np
from numpy import sqrt, arctan2 

from .BaseController import BaseController

class FeedForwardController(BaseController):
	"""Controller for the dynamical system of a car.
	
		The controller outputs are
			u1 : velocity of the car
			u2 : steering angle of the front wheels
	"""
	def __init__(self, flo, params):
		super().__init__(params)
		self.flo = flo
	
	# Overridden
	def control(self, t, x):
		"""Function of the control law.
		
		Args:
			x (ndarray, int): state vector
			t (int): time
			
		Returns:
			u		: control vector
		"""
		super().control(t, x)
		
		params = self.params
		flo_f, flo_g = self.flo

		# evaluate the planned trajectories at time t
		flo_g_t = flo_g.eval(t) # y1 = g(t)
		flo_f_t = flo_f.eval(flo_g_t[0]) # y2 = f(y1) = f(g(t))

		# setting control laws
		u1 = flo_g_t[1] * sqrt(1 + flo_f_t[1]**2)
		u2 = arctan2(params.l * flo_f_t[2], (1 + flo_f_t[1]**2)**(3/2))

		return np.array([u1, u2]).T