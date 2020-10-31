import autograd.numpy as np
from dymoesco.dynamics import dynamic_model
from dymoesco.dynamics.statespace_model import SE2

class DiffDrive(dynamic_model.ContinuousDynamicModel, SE2):
    """Differential drive wheeled robot, using velocity inputs (kinematics model)."""

    def __init__(self, radius=1, u_std=0, y_std=0):
        super().__init__(u_std, y_std)
        self.R = radius # radius of the wheels
        self.state_names = ["x", "y", "$\\theta$"]
        self.control_names = ["v", "$\\omega$"]

    def _f(self, z, u):
        x, y, theta = z
        v_lin, v_rot = u
        xdot = v_lin * np.cos(theta)
        ydot = v_lin * np.sin(theta)
        thetadot = v_rot / self.R
        return np.array([xdot, ydot, thetadot])