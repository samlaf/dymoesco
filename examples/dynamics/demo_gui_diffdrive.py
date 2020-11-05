import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as patches
from dymoesco.dynamics.diffdrive import DiffDrive
from dymoesco.utils import process_logging_level
matplotlib.use('qt5agg')

process_logging_level()

nominalu = 3
key_to_u_map = {
'up': [nominalu, 0],
'down': [-nominalu, 0],
'right': [0, -nominalu],
'left': [0, nominalu]
}

dt = 0.05
dd = DiffDrive(radius=1)
ddd = dd.discretize(dt)
ddd.gui(key_to_u_map, x0=[0,0,0])