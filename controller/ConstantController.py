#! /usr/bin/python3

# ConstantController.py

import numpy as np
from .BaseController import BaseController

class ConstantController(BaseController):
	"""Controller for the dynamical system of a car.
	
		The controller outputs are
			u1 : velocity of the car
			u2 : steering angle of the front wheels
	"""
	
	# Overridden
	def control(self, t, x, params=None):
		"""Function of the control law.
		
		Args:
			x (ndarray, int): state vector
			t (int): time
			
		Returns:
			u		: control vector
		"""
		super().control(t, x)
		
		# Linearly decreasing velocity, which is positive or zero for all time
		u1 = np.maximum(0, 1.0 - 0.1 * t)
		
		# Constant steering angle
		u2 = np.full(u1.shape, 0.25)
		
		return np.array([u1, u2]).T