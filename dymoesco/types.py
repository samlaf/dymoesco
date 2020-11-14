"""Module containing types (classes) used in dymoesco."""
class Trajectory(dict):
	""" Class to hold trajectories: t, x(t), u(t), y(t)"""
	def __init__(self, t=None, x=None, u=None, y=None):
		self.t = t
		self.x = x
		self.u = u
		self.y = y

	# From https://github.com/scipy/scipy/blob/v1.5.3/scipy/optimize/optimize.py#L82-L138
	def __getattr__(self, name):
		try:
			return self[name]
		except KeyError:
			raise AttributeError(name)

	__setattr__ = dict.__setitem__
	__delattr__ = dict.__delitem__

	def __repr__(self):
		if self.keys():
			m = max(map(len, list(self.keys()))) + 1
			return '\n'.join([k.rjust(m) + ': ' + repr(v)
							for k, v in sorted(self.items())])
		else:
			return self.__class__.__name__ + "()"

	def __dir__(self):
		return list(self.keys())