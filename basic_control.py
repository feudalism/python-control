# Basic control simulation


from car import Car, car_animation
from model import Parameters
from controller import ConstantController
from sim import Simulation, plt, plot_data

def init_model():
	model = Car(controller = ConstantController())
	model.set_params(l=0.3)
	return model
	
def set_sim_parameters():
	sim_params = Parameters()
	sim_params.t0 = 0
	sim_params.tf = 10
	sim_params.dt = 0.04
	sim_params.x0 = [0,0,0]
	return sim_params
	
def run_sim(model):
	odefunction = lambda t, x : model.ode(t, x)
	sim_params = set_sim_parameters()
	sim = Simulation(sim_params)
	return sim.simulate(odefunction)



model = init_model()

tsim, xsim = run_sim(model)

usim = model.controller.control(tsim, xsim)

plot_data(tsim, xsim, usim)
# car_animation(tsim, xsim, usim, model.params)
plt.show()