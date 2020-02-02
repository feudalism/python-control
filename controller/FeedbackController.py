#! /usr/bin/python3

# FeedbackController.py

import numpy as np
from numpy import sqrt, arctan2, sin 

from .BaseController import BaseController

class FeedbackController(BaseController):
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
		
		# state vector
		y1 = x[0]
		y2 = x[1]
		theta = x[2]
		y2_d = sin(theta)
		
		# define reference trajectories
		y1_des = flo_g_t[0]
		y1_des_d = 1 / (np.sqrt(1 + flo_f_t[1] ** 2))

		y2_des = flo_f_t[0]
		y2_des_d = flo_f_t[1] / (np.sqrt(1 + flo_f_t[1] ** 2))
		y2_des_dd = flo_f_t[2]/(1 + flo_f_t[1] ** 2)
		
		# stabilizing inputs
		w1 = y1_des_d - params.k01 * (y1 - y1_des)
		w2 = y2_des_dd - params.k12 * (y2_d - y2_des_d) - params.k02 * (y2 - y2_des)

		# setting control laws
		v_des = flo_g_t[1] * np.sqrt(1 + (flo_f_t[1]) ** 2) #desired velocity
		
		u1 = v_des * np.sqrt( w1**2 + y2_d**2)
		u2 = arctan2 (0.9 * params.l * (w2 * w1), 1)

		return np.array([u1, u2]).T