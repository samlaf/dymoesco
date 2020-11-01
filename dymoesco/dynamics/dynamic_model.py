from abc import ABC, abstractmethod
from typing import final
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import autograd.numpy as np
from dymoesco.types import Trajectory
from dymoesco.estimation.filters import EKF

class ContinuousDynamicModel(ABC):
	""" Abstract continuous-time time-invariant dynamic model class.
		If need to implement a time-varying system, need to write new class.

		All subclasses need to override _f, and can override _g."""
	def __init__(self, u_std=0, y_std=0):
		self.state_names = None
		self.control_names = None
		self.u_std = u_std
		self.y_std = y_std

	@abstractmethod
	def _f(self, x, u):
		pass

	def f(self, x, u):
		u = np.array(u, ndmin=1)
		if np.sum(u) != 0:
			u_noisy = u + np.random.normal(0, self.u_std, u.shape)
		else:
			u_noisy = u
		return self._f(x, u_noisy)

	def _g(self, x):
		return x

	def g(self, x):
		obs = self._g(x)
		if obs is None:
			return obs
		else:
			y = np.array(obs)
			y_noisy = y + np.random.normal(0, self.y_std, y.shape)
			return y_noisy

	def simulate(self, u, t_span, x0, t_eval=None):
		"""simulate calls solve_ivp and returns a Trajectory object.
		u in the returned trajectory is the underlying noiseless trajectory.
		TODO: find a way to return the noisy trajectory? (get the u_noisy out of f)."""
		sol = solve_ivp(lambda t,x: self.f(x,u(t)), t_span, x0, t_eval=t_eval)
		y = np.apply_along_axis(lambda x: self.g(x), 0, sol.y)
		return Trajectory(t=sol.t, x=sol.y, y=y, u=np.array([u(t) for t in sol.t]).T)

	def plot(self, traj, attr='x', ax=None):
		""" traj needs to be a Trajectory object (or a dictionary with t and the requested attribute (x,y or u))."""
		if ax is None:
			fig, ax = plt.subplots()
		ax.plot(traj.t, traj[attr].T)
		ax.legend([attr+str(i) for i in range(len(traj[attr]))])
		ax.set_xlabel("Time")
		return ax

	def plotx(self, traj, ax=None):
		ax = self.plot(traj, attr='x', ax=ax)
		ax.set_title("State Trajectory")
		if self.state_names is not None:
			ax.legend([self.state_names[i] for i in range(len(traj.x))])
		return ax

	def ploty(self, traj, ax=None):
		ax = self.plot(traj, attr='y', ax=ax)
		ax.set_title("Output Trajectory")
		return ax

	def plotu(self, traj, ax=None):
		ax = self.plot(traj, attr='u', ax=ax)
		ax.set_title("Control Sequence")
		if self.control_names is not None:
			ax.legend([self.control_names[i] for i in range(len(traj.u))])
		return ax

	def discretize(self, dt):
		# simple euler discretization
		cts_f = self._f
		class DiscretizedDynamicModel(DiscreteDynamicModel, *self.__class__.__bases__[1:]):
			def _f(self, x, u):
				return x + cts_f(x,u) * dt
		dd = DiscretizedDynamicModel(dt, self.u_std, self.y_std)
		dd._g = self._g
		dd.g = self.g
		return dd

class DiscreteDynamicModel(ABC):

	def __init__(self, dt, u_std=0, y_std=0):
		self.dt = dt
		self.u_std = u_std
		self.y_std = y_std

	@abstractmethod
	def _f(self, x, u):
		pass

	def f(self, x, u):
		u = np.array(u, ndmin=1)
		if np.sum(u) != 0:
			u_noisy = u + np.random.normal(0, self.u_std, u.shape)
		else:
			u_noisy = u
		return self._f(x, u_noisy)

	def _g(self, x):
		return x

	def g(self, x):
		obs = self._g(x)
		if obs is None:
			return obs
		else:
			y = np.array(obs)
			y_noisy = y + np.random.normal(0, self.y_std, y.shape)
			return y_noisy

	def make_EKF(self, Q, R):
		""" This creates an EKF using the model's _f and _g functions.
			Sometimes we might need the EKF to have a slightly different _g function
			than the model (for eg. the diffdrive range/bearing obs). In that case, use EKF direction."""
		return EKF(self._f, self._g, Q, R)