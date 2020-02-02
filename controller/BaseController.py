#! /usr/bin/python3

# controller.py

from abc import ABC, abstractmethod

import numpy as np

class BaseController(ABC):
	"""Controller for a dynamical system."""
	def __init__(self, params=None):
		self.params = params
	
	@abstractmethod
	def control(self, t, x, params=None):
		"""Function of the control law.
		
		Args:
			simdata : tuple containing the time and state
			
		Returns:
			u	: control vector
		"""
		pass