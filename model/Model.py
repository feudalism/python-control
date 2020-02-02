#! /usr/bin/python3

# Model.py

from abc import ABC, abstractmethod
from .Parameters import Parameters

class Model(ABC):
	"""Dynamical system being modelled."""
	
	def __init__(self, params=Parameters(), controller=None):
		self.params = params
		self.controller = controller
	
	@abstractmethod
	def ode(self, t, x):
		"""Dynamics of the modelled system, given as an ordinary
			differential equation.
		
		Args:
			t		: time
			x 		: state
			
		Returns:
			dxdt	: state derivative
		"""
		pass