import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from dymoesco.dynamics.doubleintegrator import DoubleIntegrator
matplotlib.use('qt5agg')

nominalu = 2
key_to_u_map = {
'up': [0,nominalu],
'down': [0,-nominalu],
'right': [nominalu,0],
'left': [-nominalu,0]
}
dt = 0.05
di = DoubleIntegrator(dim=2)
ddi = di.discretize(dt)
ddi.gui(key_to_u_map, x0=[0,0,0,0])