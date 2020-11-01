import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as patches
from dymoesco.dynamics.diffdrive import DiffDrive
from dymoesco.estimation.filters import beaconsEKF
matplotlib.use('qt5agg')

nominalu = 3.
v_std = 0.5
w_std = 0.5
u_std = np.array([v_std, w_std])
y_std = 1.0

key_to_u_map = {
'up': [nominalu, 0.],
'down': [-nominalu, 0.],
'right': [0., -nominalu],
'left': [0., nominalu]
}
dt = 0.05
dd = DiffDrive(radius=1, u_std=u_std, y_std=y_std)
ddd = dd.discretize(dt)

beacons = [[0.,0.]]
ekfdd = DiffDrive(max_radar_range=sys.float_info.max, beacons=beacons)
ekf = beaconsEKF(ekfdd._f, ekfdd._g, u_std, np.eye(2)*y_std**2)

ddd.gui(key_to_u_map, x0=[-3.,-3.,0.], ekf=ekf)