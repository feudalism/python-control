

def run():
	# # Basic control simulation
	# import sim_cases.basic_control

	# # Trajectory planning - polynomial, prototype
	# import sim_cases.traj_planning

	# # Feedforward control using a trajectory
	# import sim_cases.ff_control

	# # Feedback control
	# import sim_cases.fb_control
	
	import numpy as np
	import sympy as sp
	
	t = sp.Symbol('t')	
	params = sp.symbols('m0, m1, J1, l1, a1, g, d0, d1')
	F = sp.Symbol('F')
	
	m0, m1, J1, l1, a1, g, d0, d1 = params
	params_values = [(m0, 3.34),
					(m1, 0.3583),
					(J1, 0.0379999),
					(l1, 0.5),
					(a1, 0.43),
					(g, 9.81),
					(d0, 0.1),
					(d1, 0.006588)]
					
	q0_t = sp.Function('q0')(t)   
	q1_t = sp.Function('q1')(t) 
	
	dq0_t = q0_t.diff(t)
	dq1_t = q1_t.diff(t)
	
	ddq0_t = q0_t.diff(t, 2)
	ddq1_t = q1_t.diff(t, 2)

	
# If run directly with Python
# i.e. if not imported as a module
if __name__ == '__main__':
	run()

