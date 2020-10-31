import numpy as np
from abc import ABC, abstractmethod
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from autograd import jacobian

class Filter(ABC):

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

class EKF(Filter):
	
	def __init__(self, f, g, Q_or_ustd, R, eps=1e-10):
		self.f = f
		self.F = jacobian(f)
		self.g = g
		self.G = jacobian(g)
		if len(np.shape(Q_or_ustd))==1:
			self.Q = None
			self.u_std = np.diag(Q_or_ustd)
			self.B = jacobian(f, 1)
		else:
			self.Q = Q_or_ustd
			self.u_std = None
		self.R = R
		self.eps = eps

	def predict(self, x, P, u):
		x = np.array(x)
		u = np.array(u)
		if self.u_std is not None:
			self.Q = self.B(x,u) @ self.u_std**2 @ self.B(x,u).T
		x = self.f(x, u)
		P = self.F(x,u) @ P @ self.F(x,u).T + self.Q
		P[np.abs(P) < self.eps] = 0
		return x, P
		
	def update(self, x, P, z):
		y = z - self.g(x)
		Gx = self.G(x)
		K = P @ Gx @ np.linalg.pinv(Gx @ P @ Gx.T + self.R)
		x = x + K@y
		P = (np.eye(P.shape[0]) - K @ Gx) @ P
		P[np.abs(P) < self.eps] = 0
		return x, P

	def filter(self, traj, x0, P0):
		pass

