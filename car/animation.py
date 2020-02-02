#! /usr/bin/python3

# animation.py

import numpy as np
from numpy import sin, cos, tan

import matplotlib.pyplot as plt
import matplotlib.animation as mpla

plt.rcParams['animation.ffmpeg_path'] = 'C:\\Users\\user3\\Downloads\\ffmpeg\\bin\\ffmpeg.exe'

# LISTING_START CarAnimFunDef
def car_animation(t, x, u, params):
    """Animation function of the car-like mobile robot

    Args:
        x(ndarray): state-vector trajectory
        u(ndarray): control vector trajectory
        t(ndarray): time vector
        p(object): parameters

    Returns: None

    """
    # Setup two empty axes with enough space around the trajectory so the car
    # can always be completely plotted. One plot holds the sketch of the car,
    # the other the curve
    dx = 1.5 * params.l
    dy = 1.5 * params.l
	
    fig2, ax = plt.subplots()
    ax.set_xlim([min(min(x[:, 0] - dx), -dx),
                 max(max(x[:, 0] + dx), dx)])
    ax.set_ylim([min(min(x[:, 1] - dy), -dy),
                 max(max(x[:, 1] + dy), dy)])
				 
    ax.set_aspect('equal')
    ax.set_xlabel(r'$y_1$')
    ax.set_ylabel(r'$y_2$')

    # Axis handles
    h_x_traj_plot, = ax.plot([], [], 'b')  # state trajectory in the y1-y2-plane
    h_car, = ax.plot([], [], 'k', lw=2)    # car
# LISTING_END CarAnimFunDef

# LISTING_START CarPlotFunDef
    def car_plot(x, u):
        """Mapping from state x and action u to the position of the car elements

        Args:
            x: state vector
            u: action vector

        Returns:

        """
        wheel_length = 0.1 * params.l
        y1, y2, theta = x
        v, phi = u

        # define chassis lines
        chassis_y1 = [y1, y1 + params.l * cos(theta)]
        chassis_y2 = [y2, y2 + params.l * sin(theta)]

        # define lines for the front and rear axle
        rear_ax_y1 = [y1 + params.w * sin(theta), y1 - params.w * sin(theta)]
        rear_ax_y2 = [y2 - params.w * cos(theta), y2 + params.w * cos(theta)]
        front_ax_y1 = [chassis_y1[1] + params.w * sin(theta + phi),
                       chassis_y1[1] - params.w * sin(theta + phi)]
        front_ax_y2 = [chassis_y2[1] - params.w * cos(theta + phi),
                       chassis_y2[1] + params.w * cos(theta + phi)]

        # define wheel lines
        rear_l_wl_y1 = [rear_ax_y1[1] + wheel_length * cos(theta),
                        rear_ax_y1[1] - wheel_length * cos(theta)]
        rear_l_wl_y2 = [rear_ax_y2[1] + wheel_length * sin(theta),
                        rear_ax_y2[1] - wheel_length * sin(theta)]
        rear_r_wl_y1 = [rear_ax_y1[0] + wheel_length * cos(theta),
                        rear_ax_y1[0] - wheel_length * cos(theta)]
        rear_r_wl_y2 = [rear_ax_y2[0] + wheel_length * sin(theta),
                        rear_ax_y2[0] - wheel_length * sin(theta)]
        front_l_wl_y1 = [front_ax_y1[1] + wheel_length * cos(theta + phi),
                         front_ax_y1[1] - wheel_length * cos(theta + phi)]
        front_l_wl_y2 = [front_ax_y2[1] + wheel_length * sin(theta + phi),
                         front_ax_y2[1] - wheel_length * sin(theta + phi)]
        front_r_wl_y1 = [front_ax_y1[0] + wheel_length * cos(theta + phi),
                         front_ax_y1[0] - wheel_length * cos(theta + phi)]
        front_r_wl_y2 = [front_ax_y2[0] + wheel_length * sin(theta + phi),
                         front_ax_y2[0] - wheel_length * sin(theta + phi)]

        # empty value (to disconnect points, define where no line should be plotted)
        empty = [np.nan, np.nan]

        # concatenate set of coordinates
        data_y1 = [rear_ax_y1, empty, front_ax_y1, empty, chassis_y1,
                   empty, rear_l_wl_y1, empty, rear_r_wl_y1,
                   empty, front_l_wl_y1, empty, front_r_wl_y1]
        data_y2 = [rear_ax_y2, empty, front_ax_y2, empty, chassis_y2,
                   empty, rear_l_wl_y2, empty, rear_r_wl_y2,
                   empty, front_l_wl_y2, empty, front_r_wl_y2]

        # set data
        h_car.set_data(data_y1, data_y2)
# LISTING_END CarPlotFunDef

# LISTING_START InitFunDef
    def init():
        """Initialize plot objects that change during animation.
           Only required for blitting to give a clean slate.

        Returns:

        """
        h_x_traj_plot.set_data([], [])
        h_car.set_data([], [])
        return h_x_traj_plot, h_car
# LISTING_END InitFunDef        

# LISTING_START AnimateFunDef
    def animate(i):
        """Defines what should be animated

        Args:
            i: frame number

        Returns:

        """
        k = i % len(t)
        ax.set_title('Time (s): ' + '%.2f' % t[k], loc='left')
        h_x_traj_plot.set_xdata(x[0:k, 0])
        h_x_traj_plot.set_ydata(x[0:k, 1])
		
        car_plot(x[k, :], u[k, :])
        return h_x_traj_plot, h_car
# LISTING_END AnimateFunDef        

# LISTING_START DoAnimate
    ani = mpla.FuncAnimation(fig2, animate, init_func=init, frames=len(t) + 1,
                             interval=(t[1] - t[0]) * 1000,
                             blit=False)

    file_format = 'mp4'
    ani.save('./results/animation.'+file_format, writer='ffmpeg', fps=1 / (t[1] - t[0]))
# LISTING_END DoAnimate

    plt.show()
    return None