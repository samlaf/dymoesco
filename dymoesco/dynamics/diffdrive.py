import autograd.numpy as np
from dymoesco.dynamics import dynamic_model
from dymoesco.dynamics.statespace_model import SE2
from dymoesco.estimation.filters import EKF
import numpy

class DiffDrive(dynamic_model.ContinuousDynamicModel, SE2):
    """Differential drive wheeled robot, using velocity inputs (kinematics model)."""

    def __init__(self, radius=1, u_std=0, y_std=0, max_radar_range=1, beacons=[]):
        super().__init__(u_std, y_std)
        self.R = radius # radius of the wheels
        self.state_names = ["x", "y", "$\\theta$"]
        self.control_names = ["v", "$\\omega$"]
        self.max_radar_range = max_radar_range
        self.beacons = np.array(beacons)

    def _f(self, z, u):
        x, y, theta = z
        v_lin, v_rot = u
        xdot = v_lin * np.cos(theta)
        ydot = v_lin * np.sin(theta)
        thetadot = v_rot / self.R
        return np.array([xdot, ydot, thetadot])

    def _g(self, z, i):
        xydiff = z[:2] - self.beacons[i]
        if np.linalg.norm(xydiff) < self.max_radar_range:
            range_ = np.linalg.norm(xydiff)
            bearing = np.arctan2(xydiff[1],xydiff[0]) - z[2]
            return np.array([range_, bearing])
        return None

    def g(self, z):
        obs_dct = {}
        for i, beacon in enumerate(self.beacons):
            obs = self._g(z, i)
            if obs is not None:
                obs_dct[i] = obs + np.random.normal(0, self.y_std, obs.shape)
        return obs_dct
