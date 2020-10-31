import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from dymoesco.dynamics.singleintegrator import SingleIntegrator
#matplotlib.use('qt5agg')

nominalu = 10
key_to_u_map = {
'up': [0,nominalu],
'down': [0,-nominalu],
'right': [nominalu,0],
'left': [-nominalu,0]
}
dt = 0.10
si = SingleIntegrator(dim=2)
dsi = si.discretize(dt)
dsi.gui(key_to_u_map, x0=[0,0])