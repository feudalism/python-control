#! /usr/bin/python3

# car_model.py

import numpy as np
from numpy import sin, cos, tan

from .context import model
from model import Model

class Car(Model):
	"""Continuous time model of a car.
	
		The three states of the system are:
			x1 : horizontal position
			x2 : vertical position
			x3 : orientation in the plane
			
		The system is parametrised by the:
			l  : length of the vehicle
			
		The inputs to the system are
			u1 : velocity of the car
			u2 : steering angle of the front wheels
	"""
	
	# Overridden
	def ode(self, t, x):
		"""Dynamics of the modelled system.
		
			The dynamics and parameters are described
			by the following ODE:
			
			x1d = u1 cos x3
			x2d = u1 sin x3
			x3d = 1/l * u1 * tan u2
		
		Args:
			t		: time
			x 		: state
			
		Returns:
			dxdt	: state derivative
		"""
		params = self.params
		
		x1, x2, x3 = x
		u1, u2 = self.controller.control(t, x)
		
		dxdt = np.array([ 	u1 * cos(x3),
							u1 * sin(x3),
							1 / params.l * u1 * tan(u2)])
							
		return dxdt
		
	def set_params(self, l):
		self.params.l = l
		self.params.w = l * 0.3