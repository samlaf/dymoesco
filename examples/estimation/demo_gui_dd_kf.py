import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as patches
from dymoesco.dynamics.diffdrive import DiffDrive
from dymoesco.estimation.filters import beaconsEKF
from dymoesco.utils import process_logging_level
matplotlib.use('qt5agg')

process_logging_level()

nominalu = 3.
v_std = 1.0
w_std = 1.0
u_std = np.array([v_std, w_std])
y_std = np.array([0.5, 0.25])

key_to_u_map = {
'up': [nominalu, 0.],
'down': [-nominalu, 0.],
'right': [0., -nominalu],
'left': [0., nominalu]
}

x0=[-5., -5., 0.]
beacons = [[2.,2.], [-2,2], [2,-2],[-2,-2]]
max_radar_range = 3
dt = 0.05

dd = DiffDrive(radius=1, max_radar_range=max_radar_range, u_std=u_std, y_std=y_std, beacons=beacons, name='gt')
ddd = dd.discretize(dt)

ekfdd = DiffDrive(max_radar_range=sys.float_info.max, beacons=beacons, name='ekf')
ekfddd = ekfdd.discretize(dt)
ekf = beaconsEKF(ekfddd._f, ekfddd._g, u_std, np.eye(2)*y_std**2)

ddd.gui(key_to_u_map, x0=x0, ekf=ekf)