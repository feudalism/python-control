# Basic control simulation

from car import Car, car_animation
from planner import PolynomialPlanner
from controller import FeedForwardController

from model import Parameters
from sim import Simulation, plt, plot_data

import numpy as np
from numpy import tan

def init_model():
	model = Car()
	model.set_params(l=0.3)
	return model
	
def get_sim_parameters():
	sim_params = Parameters()
	sim_params.t0 = 0
	sim_params.tf = 10
	sim_params.dt = 0.04
	sim_params.x0 = [0,0,0]
	sim_params.xf = [5, 5, 0]
	return sim_params
	
def get_traj_parameters():
	sim_params = get_sim_parameters()

	traj_params = Parameters()
	traj_params.t0 = sim_params.t0 + 1
	traj_params.tf = sim_params.tf - 1

	# boundary conditions for y1
	traj_params.Y1A = np.array([sim_params.x0[0], 0])
	traj_params.Y1B = np.array([sim_params.xf[0], 0])

	# boundary conditions for y2
	traj_params.Y2A = np.array([sim_params.x0[1], tan(sim_params.x0[2]), 0])
	traj_params.Y2B = np.array([sim_params.xf[1], tan(sim_params.xf[2]), 0])
	
	return traj_params
	
def generate_trajectory():
	traj_params = get_traj_parameters()

	flo_f = PolynomialPlanner(traj_params.Y2A,
			traj_params.Y2B,
			traj_params.Y1A[0],
			traj_params.Y1B[0],
			2)
	flo_g = PolynomialPlanner(traj_params.Y1A,
			traj_params.Y1B,
			traj_params.t0,
			traj_params.tf,
			1)
	
	return flo_f, flo_g

def run_sim(model):
	odefunction = lambda t, x : model.ode(t, x)
	sim_params = get_sim_parameters()
	sim = Simulation(sim_params)
	return sim.simulate(odefunction)

model = init_model()

flo = generate_trajectory()
model.controller = FeedForwardController(flo, model.params)

tsim, xsim = run_sim(model)

usim = np.zeros([len(tsim),2])
for i in range(0, len(tsim)):
    usim[i] = model.controller.control(tsim[i], xsim[i])

plot_data(tsim, xsim, usim)
# car_animation(tsim, xsim, usim, model.params)
plt.show()