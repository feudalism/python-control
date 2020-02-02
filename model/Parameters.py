#! /usr/bin/python3

# Parameters.py

class Parameters(object):
	"""Structure to store parameters as attributes."""
	
	def append(self, params):
		new_attributes = dict(params)
		self.__dict__.update(new_attributes)
		
	def __iter__(self):
		""" Allow object to be iterable """
		for attr, value in self.__dict__.items():
			yield attr, value