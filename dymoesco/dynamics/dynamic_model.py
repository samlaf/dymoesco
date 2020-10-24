from abc import ABC, abstractmethod

class ContinuousDynamicModel(ABC):
	""" Abstract continuous-time time-invariant dynamic model class.
		If need to implement a time-varying system, need to write new class."""

	@abstractmethod
	def f(self, x, u):
		pass

	def g(self, x):
		return x

	def discretize(self, dt):
		# simple euler discretization
		ctsf = self.f
		class DiscretizedDynamicModel(DiscreteDynamicModel):
			def f(self, x, u):
				return x + ctsf(x,u) * dt
		return DiscretizedDynamicModel()

class DiscreteDynamicModel(ABC):

	@abstractmethod
	def f(self, x, u):
		pass

	def g(self, x):
		return x