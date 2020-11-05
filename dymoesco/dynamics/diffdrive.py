import autograd.numpy as np
from dymoesco.dynamics import dynamic_model
from dymoesco.dynamics.dynamic_model import DiscreteDynamicModel
from dymoesco.dynamics.statespace_model import SE2
from dymoesco.estimation.filters import EKF
from dymoesco.utils import modpi2pi
import logging

class DiffDrive(dynamic_model.ContinuousDynamicModel, SE2):
	r"""Differential drive wheeled robot, using velocity inputs (kinematics model).

	DiffDrive is an implementation of a differential drive wheeled robot happening
	on the SE(2) state-space with variables :math:`(x,y,\theta)`:
	
	.. math::

		\dot{x} &= v \cos\theta \\
		\dot{y} &= v \sin\theta \\
		\dot{\theta} &= w / \text{radius}

	DiffDrive also has a range and bearing sensor, which has a max range of
	`max_radar_range` and the bearing sensor works irrespective of the position
	of the beacon (360deg coverage).

	Parameters
	----------
	radius : float
		radius of the wheels
	u_std : float or list of 3 floats
		control noise standard deviation. Can also give different standard deviations
		for each state (x,y theta).
	y_std : float or list of 2 floats
		sensor noise standard deviation. Can also give a different standard deviations
		for range and bearing.
	max_radar_range : float
		maximum range for which the range sensor will return an observation
	beacons : list of tuples
		list of beacon (x,y)-positions, one for each beacon. Beacons are sensed
		by the range-bearing sensor (_g/g methods)

	"""
	def __init__(self, radius=1, u_std=0, y_std=0, max_radar_range=1, beacons=[], name=None):
		super().__init__(u_std, y_std)
		self.R = radius
		self.state_names = ["x", "y", "$\\theta$"]
		self.control_names = ["v", "$\\omega$"]
		self.max_radar_range = max_radar_range
		self.beacons = np.array(beacons)
		self.name = name

	def _f(self, z, u):
		x, y, theta = z
		v_lin, v_rot = u
		xdot = v_lin * np.cos(theta)
		ydot = v_lin * np.sin(theta)
		thetadot = v_rot / self.R
		return np.array([xdot, ydot, thetadot])

	def _g(self, z, i):
		xydiff = self.beacons[i] - z[:2]
		if np.linalg.norm(xydiff) <= self.max_radar_range:
			range_ = np.linalg.norm(xydiff)
			bearing = modpi2pi(np.arctan2(xydiff[1],xydiff[0]) - z[2])
			logging.info(f"DiffDrive {self.name} observations: range {range_} / bearing {bearing}")
			return np.array([range_, bearing])
		return None

	def g(self, z):
		"""Observation function for beacons-based range/bearing sensor.

		Note how we overrode :meth:`dymoesco.dynamics.dynamic_model.ContinuousDynamicModel.g` to get
		the desired behavior.
		"""
		obs_dct = {}
		for i, beacon in enumerate(self.beacons):
			obs = self._g(z, i)
			if obs is not None:
				obs_dct[i] = obs + np.random.normal(0, self.y_std, obs.shape)
		return obs_dct

	def discretize(self, dt):
		# need to overwrite discretize to mod theta.
		cts_f = self._f
		class DiscretizedDynamicModel(DiscreteDynamicModel, *self.__class__.__bases__[1:]):
			def _f(self, x, u):
				x_p = x + cts_f(x,u) * dt
				return np.array([x_p[0], x_p[1], np.mod(x_p[2], 2*np.pi)])
		dd = DiscretizedDynamicModel(dt)
		dd.__dict__ = self.__dict__
		dd.dt = dt
		dd._g = self._g
		dd.g = self.g
		return dd
