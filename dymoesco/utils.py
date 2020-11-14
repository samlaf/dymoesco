"""Module for helper functions."""
import argparse
import logging
import numpy as np

def modpi2pi(x):
	r"""mod x such that it is between :math:`[-\pi, \pi)`.

		>>> modpi2pi(np.pi)
		- 3.141592653589793
		>>> modpi2pi(2*np.pi)
		0.0
		>>> modpi2pi(3*np.pi)
		- 3.141592653589793
	"""
	return modrange(x, -np.pi, np.pi)

def modrange(x, low, high):
	"""mod x such that it is between [low, high).

		>>> modrange(2.5, 1, 2)
		1.5
		>>> modrange(3., 1, 2)
		1.0
	"""
	spread = high - low
	diff = x - low
	mod = diff % spread
	return low + mod

def parse_and_setup_logging():
	"""uses argparse to determine which logging level to use.

	scripts that use parse_and_setup_logging can now be called with `--loglevel=<LEVEL>` and 
	'--logger=<LOGGER>' as argument.

	Example
	-------
	:code:`python3 script.py --loglevel=INFO logger=prediction` will output info logs from 
	the Kalman Filter's prediction logger to stdout (as opposed to only warnings
	by default). See :meth:`dymoesco.dynamics.statespace_models.Rn.gui` for an example `logging.info`.
	
	Notes
	-----
	See	https://docs.python.org/3/howto/logging.html#logging-to-a-file (a bit lower down) for more 
	details about logging.
	"""
	logparser = argparse.ArgumentParser(description='Process logging level input.')
	logparser.add_argument('--loglevel', type=str, help='logging level', default='WARNING')
	logparser.add_argument('--logger', type=str, nargs='*', default=None,
								help='which logger(s) to use (prediction, update, gui) default=all')

	args = logparser.parse_args()

	# set loglevel
	numeric_level = getattr(logging, args.loglevel.upper(), None)
	if not isinstance(numeric_level, int):
		raise ValueError('Invalid log level: %s' % numeric_level)
	logging.basicConfig(level=numeric_level)

	logger_names = ['prediction', 'update', 'gui', 'diffdrive'] # add to this list as loggers are added to the code
	# set logger
	if args.logger is not None:
		for logger_name in logger_names:
			if logger_name not in args.logger:
				logging.getLogger(logger_name).disabled = True

def turn_off_logging_wrapper(f):
	"""wrapper to return logging level to WARNING while executing function.

	See :func:`dymoesco.estimation.filters.calc_jacobian` for an example."""	
	def wrapper(*args, **kwargs):
		logger = logging.getLogger()
		loglevel = logger.getEffectiveLevel()
		logger.setLevel(logging.WARNING)
		ret = f(*args, **kwargs)
		logger.setLevel(loglevel)
		return ret
	return wrapper