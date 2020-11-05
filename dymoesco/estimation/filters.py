"""The :mod:`~dymoesco.estimation.filters` module contains the
:class:`~dymoesco.estimation.filters.Filter` abstract class that is subclassed
by specific filters such as :class:`~dymoesco.estimation.filters.KalmanFilter` and
:class:`~dymoesco.estimation.filters.EKF`. Filters implement the predict and update
methods which are used to "denoise" noisy state-space trajectories.
"""

import numpy as np
from abc import ABC, abstractmethod
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from autograd import jacobian
from dymoesco.utils import turn_off_logging_wrapper
import logging

class Filter(ABC):
	"""An abstract class for enforcing the predict/update filtering API.

	The Filter abstract class allows filtering algorithms to be 
	represented and plotted in Python.  It is intended as a parent
	class for a set of subclasses that are used to implement specific
	structures and operations for different types of dynamical systems.

	"""
	@abstractmethod
	def predict(self, x, P, u):
		pass

	@abstractmethod
	def update(self, x, P, z):
		pass

	def plot(self, xs, Ps, ax=None, cov_step=5, color=None, **kwargs):
		def get_cov_ellipse(x, P, color=None):
			w, v = np.linalg.eig(P)
			angle = np.arctan2(v[0][1], v[0][0])
			width = np.sqrt(w[0])
			height = np.sqrt(w[1])
			return Ellipse(x, width, height, angle, fill=None, color=color)
		if ax is None:
			fig, ax = plt.subplots(subplot_kw={"box_aspect": 1})
		ax.plot(*xs.T, color=color, **kwargs)
		for i in range(0, len(xs), cov_step):
			ax.add_patch(get_cov_ellipse(xs[i],Ps[i],color=color))
		return ax

class KalmanFilter(Filter):
	r"""The well-known algorithm for filtering linear dynamical systems.

	The KalmanFilter class implements the predict and update abstract methods
	of `Filter` for linear dynamical systems 

	.. math::
		x_{k+1} &= Ax_k + Bu_k + v \\
		y_k &= Cu_k + w

	Parameters
	----------
	A : numpy matrix or equivalent list of list
		Dynamics matrix from :math:`x_{k+1}=Ax_k+Bu_k+v`.
	B : numpy matrix or equivalent list of list
		Control matrix from :math:`x_{k+1}=Ax_k+Bu_k+v`.
	C : numpy matrix or equivalent list of list
		Observation matrix from :math:`y_k=Cu_k`.
	Q : numpy matrix or equivalent list of list
		Prediction noise covariance matrix for :math:`v \sim N(0,Q)`.
	R : numpy matrix or equivalent list of list
		Observation noise covariance matrix for :math:`w \sim N(0,Q)`.
	"""
	def __init__(self, A, B, C, Q, R):
		self.A = A
		self.B = B
		self.C = C
		self.Q = Q
		self.R = R

	def predict(self, x, P, u):
		x = self.A @ x + self.B @ u
		P = self.A @ P @ self.A.T + self.Q
		return x, P

	def update(self, x, P, z):
		y = z - self.C @ x
		K = P @ self.C.T @ np.linalg.pinv(self.C @ P @ self.C.T + self.R)
		x = x + K @ y
		P = (np.eye(P.shape[0]) - K @ self.C) @ P
		return x, P

@turn_off_logging_wrapper
def calc_jacobian(f, x, *args, use_autograd=False, eps = 1e-3):
	r""" Calculates the jacobian of f wrt x.

	Either using autograd or finite difference.
	autograd is returning NaN for the beacon observation in diffdrive...
	so need to use finite diff there. Otherwise use autograd.

	"""
	if use_autograd:
		J = jacobian(f)(x, *args)
	else:
		J = np.zeros((len(f(x, *args)), len(x)))
		for i in range(len(x)):
			eps_i = np.zeros_like(x)
			eps_i[i] = eps
			J[:,i] = (f(x+eps_i, *args) - f(x, *args))/eps
	return J

class EKF(Filter):

	def __init__(self, f, g, Q_or_ustd, R, eps=1e-10, use_autograd=False):
		self.f = f
		self.F = jacobian(f)
		self.g = g
		if len(np.shape(Q_or_ustd))==1:
			self.Q = None
			self.u_std = np.diag(Q_or_ustd)
			self.B = jacobian(f, 1)
		else:
			self.Q = Q_or_ustd
			self.u_std = None
		self.R = R
		self.eps = eps
		self.use_autograd = use_autograd

	def predict(self, x, P, u):
		x = np.array(x)
		u = np.array(u)
		if self.u_std is not None:
			self.Q = self.B(x,u) @ self.u_std**2 @ self.B(x,u).T
		x = self.f(x, u)
		P = self.F(x,u) @ P @ self.F(x,u).T + self.Q
		P[np.abs(P) < self.eps] = 0
		return x, P
		
	def _update(self, x, P, z):
		# _update assumes z is a np.array of scalar observations
		y = z - self.g(x)
		Gx = calc_jacobian(self.g, x, use_autograd=self.use_autograd)
		K = P @ Gx.T @ np.linalg.pinv(Gx @ P @ Gx.T + self.R)
		x = x + K@y
		P = (np.eye(P.shape[0]) - K @ Gx) @ P
		P[np.abs(P) < self.eps] = 0
		return x, P

	def update(self, x, P, z):
		# If z needs to be anything more complicated,
		# we can subclass EKF and write a specific update function
		# see dymoesco/dynamics/diffdrive.py for an example.
		if z is None:
			return x, P
		return self._update(x, P, z)

	def filter(self, traj, x0, P0):
		pass

class beaconsEKF(EKF):

	def __init__(self, f, g, Q_or_ustd, R, eps=1e-10, use_autograd=False):
		super().__init__(f, g, Q_or_ustd, R, eps=1e-10, use_autograd=False)

	def update(self, x, P, z_dct):
		for i, obs in z_dct.items():
			x, P = self._update(x, P, obs, i)
		return x, P

	def _update(self, x, P, z, i):
		# _update assumes z is a np.array of scalar observations
		y = z - self.g(x, i)
		logging.debug(f"observation z: {z}")
		logging.debug(f"innovation y: {y}")
		Gx = calc_jacobian(self.g, x, i, use_autograd=self.use_autograd)
		K = P @ Gx.T @ np.linalg.pinv(Gx @ P @ Gx.T + self.R)
		logging.debug(f"Kalman gain K: \n {K}")
		x = x + K@y
		P = (np.eye(P.shape[0]) - K @ Gx) @ P
		P[np.abs(P) < self.eps] = 0
		return x, P
