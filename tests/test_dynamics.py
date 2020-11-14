from dymoesco.dynamics.dynamic_models import ContinuousDynamicModel, DiscreteDynamicModel, DiffDrive
from dymoesco.control.misc_controllers import generate_smooth_control
from dymoesco.types import Trajectory
import unittest
import numpy as np

class TestDynamicsModel(unittest.TestCase):
	"""These tests make sure that ContinuousDynamicModel and DiscreteDynamicModel
	are abstract classes that can't be implemented."""
	def setUp(self):
		pass

	def tearDown(self):
		pass

	def test_abstract_ContinuousDynamicModel(self):
		"""Make sure ContinuousDynamicModel is abstract and can't be instantiated."""
		with self.assertRaises(TypeError):
			ContinuousDynamicModel()

	def test_abstract_DiscreteDynamicModel(self):
		"""Make sure DiscreteDynamicModel is abstract and can't be instantiated."""
		with self.assertRaises(TypeError):
			DiscreteDynamicModel()

class TestDiffDrive(unittest.TestCase):

	def setUp(self):
		self.dd = DiffDrive(u_std=1)
		self.ddbeacons = DiffDrive(beacons=[[0,0]], y_std=1)

	def tearDown(self):
		pass

	def test__f(self):
		"""_f returns np array with diff drive kinematic eqns."""
		self.assertTrue(type(self.dd._f([0,0,0],[1,1])) is np.ndarray)
		self.assertTrue(np.array_equal(self.dd._f([0,0,0],[1,1]),
										[1,0,1]))

	def test_f(self):
		"""f should return a noisy version of _f when u is not [0,0]."""
		self.assertTrue(np.array_equal(self.dd.f([0,0,0],[0,0]),
										[0,0,0]))
		self.assertFalse(np.array_equal(self.dd.f([0,0,0],[1,1]),
										[1,0,1]))

	def __g(self):
		"""_g(z,i) should return a noiseless range/bearing observation for beacon i."""
		self.assertTrue(np.array_equal(self.ddbeacons._g([0,0,0], 0),
										[0,0]))
		self.assertTrue(np.array_equal(self.ddbeacons._g([0,0,1],0),
										[0,-1]))
		self.assertTrue(np.array_equal(self.ddbeacons._g([1,0,-np.pi],0),
										[1,0]))


	def test_g(self):
		"""g should return a dictionary of noisy range/bearing observation."""
		self.assertTrue(self.dd.g([0,0,0]) == {})
		self.assertTrue(len(self.ddbeacons.g([0,0,0])) == 1)
		self.assertFalse(np.array_equal(self.ddbeacons.g([0,0,0])[0],
										[0,0]))
	def test_discretize(self):
		"""discretize should return a DiscreteDynamicModel."""
		dt=0.1
		ddd = self.dd.discretize(dt=dt)
		self.assertTrue(isinstance(ddd, DiscreteDynamicModel))
		self.assertTrue(np.array_equal(ddd._f([0,0,0],[1,1]),
										[dt,0,dt]))
		self.assertTrue(ddd.__dict__ == self.dd.__dict__)
		self.assertFalse(np.array_equal(ddd.f([0,0,0],[1,0]),
										[dt,0,0]))

	def test_simulate(self):
		"""Make sure simulate returns a Trajectory object."""
		traj = self.dd.simulate(lambda t: [1,1], (0,1), [0,0,0], (0,1))
		self.assertTrue(isinstance(traj, Trajectory))


if __name__ == '__main__':
	unittest.main()