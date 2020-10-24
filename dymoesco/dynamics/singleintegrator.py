from dynamics.dynamic_model import ContinuousDynamicModel

class SingleIntegrator(ContinuousDynamicModel):

	def __init__(self, dim=1):
		self.dim = dim

	def f(self, x, u):
		assert len(u) == self.dim
		return u
