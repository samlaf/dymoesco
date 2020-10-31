from dymoesco.dynamics.dynamic_model import ContinuousDynamicModel
from dymoesco.dynamics.statespace_model import Rn

class SingleIntegrator(ContinuousDynamicModel, Rn):

	def __init__(self, dim=1):
		super().__init__()
		self.dim = dim

	def _f(self, x, u):
		assert len(u) == self.dim
		return u
