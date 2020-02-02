import numpy as np
import scipy.integrate as sci

from .events import event

class Simulation(object):
	def __init__(self, sim_params, method='RK45'):
		self.t0 = sim_params.t0
		self.tf = sim_params.tf
		self.dt = sim_params.dt
		self.x0 = sim_params.x0
		self.method = method
		
	def simulate(self, odefunction):
		tt = self.get_time_vector()
		
		sol = sci.solve_ivp(odefunction,
			(self.t0, self.tf),
			self.x0,
			method=self.method,
			t_eval=tt,
			# events=event
			)
			
		return sol.t, sol.y.T
		
	def get_time_vector(self):
		return np.arange(self.t0, self.tf + self.dt, self.dt)
		
