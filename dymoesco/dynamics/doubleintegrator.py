from dymoesco.dynamics.dynamic_model import ContinuousDynamicModel
from dymoesco.dynamics.statespace_model import Rn
import autograd.numpy as np

class DoubleIntegrator(ContinuousDynamicModel, Rn):

	def __init__(self, dim=1):
		self.dim = dim

	def _f(self, x, u):
		x = np.array(x).reshape(-1)
		u = np.array(u).reshape(-1) # in case u is an int
		assert len(x) == 2*self.dim
		assert len(u) == self.dim
		p = x[self.dim:]
		qdot = p
		pdot = u
		return np.append(qdot, pdot)
