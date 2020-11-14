"""State space classes which dymoesco models can inherit from to get plotting facilities."""
from abc import ABC
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse, Circle
import numpy as np
import logging
from dymoesco.estimation.filters import EKF

class StateSpace(ABC):
	"""Dynamics can happen on different state spaces. Each state space will have its own class to hold
	methods for its own peculiarities, such as plotting, printing information, etc."""

class Rn(StateSpace):

	def plot_phase(self, traj, dim1=0, dim2=1, ax=None, color=None, **kwargs):
		assert dim1 < len(traj.x), "traj doesn't contain a dimension {}".format(dim1)
		assert dim2 < len(traj.x), "traj doesn't contain a dimension {}".format(dim2)
		if ax is None:
			fig, ax = plt.subplots()
		dim1=0;dim2=1
		ax.plot(traj.x[dim1], traj.x[dim2], color=color, **kwargs)
		ax.scatter(traj.x[dim1][0], traj.x[dim2][0], label="init", color=color)
		for n in [len(traj.t)//4, len(traj.t)//2, len(traj.t)//4*3]:
			ax.arrow(traj.x[dim1][n], traj.x[dim2][n], traj.x[dim1][n+1] - traj.x[dim1][n], traj.x[dim2][n+1] - traj.x[dim2][n], 
						lw=0, length_includes_head=True, head_width=.04, color=color)
		ax.legend()
		ax.set_xlabel(f"x{dim1}")
		ax.set_ylabel(f"x{dim2}")

	def animate(self, traj, dim1=0, dim2=1, tail_len=3, ax=None):
		if ax is None:
			fig, ax = plt.subplots()
		ax.set_xlabel(f"x{dim1}")
		ax.set_ylabel(f"x{dim2}")

		xdata, ydata = [], []
		ln, = plt.plot([], [], 'ro')
		time_template = 'time = %.1fs'
		time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

		def init():
			xylim = max(1, traj.x[dim1].max(), (-traj.x[dim1]).max(), traj.x[dim2].max(), (-traj.x[dim2]).max())
			ax.set_xlim(-xylim,xylim)
			ax.set_ylim(-xylim,xylim)
			return ln, time_text

		def update(frame):
			xdata.append(traj.x[dim1][frame])
			ydata.append(traj.x[dim2][frame])
			if len(xdata) > tail_len:
				xdata.pop(0)
				ydata.pop(0)
			ln.set_data(xdata, ydata)
			time_text.set_text(time_template % (frame*dt))
			return ln, time_text

		dt = traj.t[1] - traj.t[0]
		ani = FuncAnimation(fig, update, frames=len(traj.t), init_func=init, blit=True, interval=1000*dt)
		return ani

	def gui(self, key_to_u_map, x0, dim1=0, dim2=1):
		r""" gui for Rn statespace models.

		Starts an interactive gui for the dynamic_models in `self`, which is controlled using the arrows
		(which binding defined by key_to_u_map).

		Parameters
		----------
		key_to_u_map : dict
			key_to_u_map binds 'up', 'down', 'right', 'left' to controls.

		Examples
		--------
		::

			key_to_u_map =  {'up': [0,1], 'down': [0,-1], 'right': [1,0], 'left': [-1,0]}

		Notes
		-----
		self needs to be a DiscreteDynamicModel.

		"""
		fig, ax = plt.subplots()
		ax.set_title('Drive by pressing the keyboard arrows.')
		ax.set_xlim([-10,10])
		ax.set_ylim([-10,10])
		ax.get_yaxis().set_visible(False)
		ax.get_xaxis().set_visible(False)

		u0 = np.zeros_like(key_to_u_map['up'])
		class Dot(): pass
		Dot.state = x0
		Dot.nextu = u0

		dot, = ax.plot(Dot.state[0], Dot.state[1], 'o')
		def on_timer(dot, Dot):
			Dot.state = self.f(Dot.state, Dot.nextu)
			Dot.nextu = u0
			dot.set_data(Dot.state[0], Dot.state[1])
			fig.canvas.draw()
		timer = fig.canvas.new_timer(interval=1000*self.dt, callbacks=[(on_timer, [dot, Dot], {})])
		timer.start()

		def press(event, Dot):
			if event.key == 'up':
				Dot.nextu = key_to_u_map['up']
			elif event.key == 'down':
				Dot.nextu = key_to_u_map['down']
			elif event.key == 'right':
				Dot.nextu = key_to_u_map['right']
			elif event.key == 'left':
				Dot.nextu = key_to_u_map['left']
		fig.canvas.mpl_connect('key_press_event', lambda event: press(event, Dot))

		plt.show()

class SE2(StateSpace):

	def animate(self, traj, ax=None):
		if ax is None:
			fig, ax = plt.subplots()
		ax.set_xlabel("x")
		ax.set_ylabel("y")

		car = Ellipse((0, 0), 2.0, 1.0, facecolor='y')
		time_template = 'time = %.1fs'
		time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

		def init():
			xylim = max(10, traj.x[0].max(), (-traj.x[0]).max(), traj.x[1].max(), (-traj.x[1]).max())
			ax.set_xlim(-xylim,xylim)
			ax.set_ylim(-xylim,xylim)
			ax.add_patch(car)
			return car, time_text

		def update(frame):
			car.set_center([traj.x[0][frame],traj.x[1][frame]])
			car.angle = np.rad2deg(traj.x[2][frame])
			time_text.set_text(time_template % (frame*dt))
			return car, time_text

		dt = traj.t[1] - traj.t[0]
		ani = FuncAnimation(fig, update, frames=len(traj.t), init_func=init, blit=True, interval=1000*dt)
		return ani

	def gui(self, key_to_u_map, x0, ekf=None, show_pred = False, update_every_n_pred=10):

		guilogger = logging.getLogger('gui')
		fig, ax = plt.subplots()
		ax.set_title('Drive by pressing the keyboard arrows.')
		ax.set_xlim([-10,10])
		ax.set_ylim([-10,10])
		ax.get_yaxis().set_visible(False)
		ax.get_xaxis().set_visible(False)

		u0 = np.zeros_like(key_to_u_map['up'])
		class Car():
			def __init__(self, x0, cov = None, color=None, name=None):
				self.name = name
				self.state = x0
				self.ellipse = Ellipse(x0[:-1], 2.0, 1.0, x0[-1], color=color)
				ax.add_patch(self.ellipse)
				self.nextu = u0
				self.cov = cov
				self.cov_ellipse = None
				if cov is not None:
					self.cov_ellipse = Ellipse((0,0), 1.0, 1.0, 0, fill=None, color=color)
					ax.add_patch(self.cov_ellipse)

			def update(self, state, cov=None):
				guilogger.info(f"Car {self.name} state: {state}")
				self.state = state
				self.cov = cov
				self.ellipse.set_center(state[:-1])
				self.ellipse.angle = np.rad2deg(state[-1])
				if cov is not None:
					self.cov = cov
					w, v = np.linalg.eig(cov[:2,:2])
					self.cov_ellipse.set_angle(np.rad2deg(np.arctan2(v[1][0], v[0][0])))
					self.cov_ellipse.set_width(np.sqrt(w[0]))
					self.cov_ellipse.set_height(np.sqrt(w[1]))
					self.cov_ellipse.set_center(state)
				if cov is None:
					self.nextu = u0

		# ground-truth Car
		gtCar = Car(x0, color='black', name="ground-truth")
		# plot its beacons if there are any
		for bx,by in self.beacons:
			ax.plot(bx, by, '*', color='y')
			ax.add_patch(Circle((bx,by), self.max_radar_range, fill=False, linestyle='--', color='y'))


		# ekf
		if ekf is not None:
			# Pred only
			if show_pred:
				predCar = Car(x0, cov=5*np.eye(3), color='yellow', name="pred-only")
			# Pred + Update
			EKFCar = Car(x0, cov=5*np.eye(3), color='green', name="ekf")

		def on_timer(Car):
			# EKF Update
			if ekf is not None:
				# Pred only
				if show_pred:
					predCar.update(*ekf.predict(predCar.state, predCar.cov, Car.nextu))
				# Pred + Update
				EKFCar.update(*ekf.predict(EKFCar.state, EKFCar.cov, Car.nextu))
				EKFCar.update(*ekf.update(EKFCar.state, EKFCar.cov, self.g(Car.state)))

			# Car Update
			Car.update(self.f(Car.state, Car.nextu))

			fig.canvas.draw()
		timer = fig.canvas.new_timer(interval=1000*self.dt, callbacks=[(on_timer, [gtCar], {})])
		timer.start()

		# if ekf is not None:
		# 	def on_timer2(EKFCar):
		# 		EKFCar.update(*ekf.update(EKFCar.state, EKFCar.cov, self.g(gtCar.state)))
		# 	timer2 = fig.canvas.new_timer(interval=1000*self.dt*update_every_n_pred, 
		# 									callbacks=[(on_timer2, [EKFCar], {})])
		# 	timer2.start()

		def press(event, Car):
			if event.key == 'up':
				Car.nextu = key_to_u_map['up']
			elif event.key == 'down':
				Car.nextu = key_to_u_map['down']
			elif event.key == 'right':
				Car.nextu = key_to_u_map['right']
			elif event.key == 'left':
				Car.nextu = key_to_u_map['left']
		fig.canvas.mpl_connect('key_press_event', lambda event: press(event, gtCar))

		plt.show()