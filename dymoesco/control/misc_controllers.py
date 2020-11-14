"""Control sequence generators."""
import numpy as np
from scipy import signal
from scipy.interpolate import interp1d

def generate_smooth_control(umin, umax, t_range, dt, noise_std = 0.0, rng=None, output="function"):
	"""generate a low-pass filtered control sequence.
	
	Parameters
	----------
	umin : float or list of floats
		minimum control value for each dimensions specified.
	umax : float or list of floats
		maximum control value for each dimensions specified.
	t_range : (float, float) tuple
		tmin and tmax in between which we want to generate controls.
	dt : float
		time delta

	Returns
	-------
	function or array:
		output is either a function (of time) or an array which we can index, depending on `output` parameter.
	"""
	if rng is None:
		rng = np.random.default_rng()
	umin = np.array(umin, ndmin=1).reshape(-1,1); umax = np.array(umax, ndmin=1).reshape(-1,1);
	t = np.arange(t_range[0], t_range[1]+dt, dt)
	size = (len(umin), len(t))
	u_rough = rng.uniform(umin, umax, size)
	sos = signal.butter(10, 0.1, output='sos')
	u_array = signal.sosfilt(sos, u_rough)
	noise = rng.normal(0, noise_std, size)
	noisy_u_array = u_array + noise
	if output == "function":
		u = interp1d(t, noisy_u_array)
	elif output == "array":
		u = noisy_u_array
	else:
		raise ValueError("output parameter should be one of 'function' or 'array'.")
	return u