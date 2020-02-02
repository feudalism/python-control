#! /usr/bin/python3

# plotter.py

import numpy as np
import matplotlib.pyplot as plt

def plot_data(t, x, u, fig_width_in_cm=12, fig_height_in_cm=16, save=False):
	fig1, axes = plt.subplots(3)
	ax1, ax2, ax3 = axes
	
	fig1.set_size_inches(fig_width_in_cm / 2.54, fig_height_in_cm / 2.54)
	
	ax1.plot(t, x[:,0], label='$y_1(t)$', lw=1, color='r')
	ax1.plot(t, x[:,1], label='$y_2(t)$', lw=1, color='b')
	ax2.plot(t, np.rad2deg(x[:,2]), label=r'$\theta(t)$', lw=1, color='g')
	
	ax3.plot(t, np.rad2deg(u[:,0]), label=r'$v(t)$', lw=1, color='r')
	ax3.tick_params(axis='y', colors='r')
	
	ax33 = ax3.twinx()
	ax33.plot(t, np.rad2deg(u[:,1]), label=r'$\phi(t)$', lw=1, color='b')
	ax33.spines["left"].set_color('r')
	ax33.spines["right"].set_color('b')
	ax33.tick_params(axis='y', colors='b')
	
	ax1.set_title('Position coordinates')
	ax1.set_ylabel(r'm')
	
	ax2.set_title('Orientation')
	ax2.set_ylabel(r'deg')
	
	ax3.set_title('Velocity / Steering angle')
	ax3.set_ylabel(r'm/s')
	ax33.set_ylabel(r'deg')
	
	ax_settings(axes)
	
	li3, lab3 = ax3.get_legend_handles_labels()
	li33, lab33 = ax33.get_legend_handles_labels()
	ax3.legend(li3 + li33, lab3 + lab33, loc=0)
	
	plt.tight_layout()
	
	if save:
		plt.savefig('./results/state_trajectory.pdf')
	
	return None	

def ax_settings(axes):
	for ax in axes:
		ax.set_xlabel(r't in s')
		ax.grid(True)
		ax.legend()
