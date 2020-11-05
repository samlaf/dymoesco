import argparse
import logging
import numpy as np

def modpi2pi(x):
	return modrange(x, -np.pi, np.pi)

def modrange(x, low, high):
	spread = high - low
	diff = x - low
	mod = diff % spread
	return low + mod

def process_logging_level():
	logparser = argparse.ArgumentParser(description='Process logging level input.')
	logparser.add_argument('--log', type=str, help='logging level', default='WARNING')

	args = logparser.parse_args()
	# https://docs.python.org/3/howto/logging.html#logging-to-a-file (a bit lower down)
	numeric_level = getattr(logging, args.log.upper(), None)
	if not isinstance(numeric_level, int):
		raise ValueError('Invalid log level: %s' % loglevel)
	logging.basicConfig(level=numeric_level)

def turn_off_logging_wrapper(f):
	def wrapper(*args, **kwargs):
		logger = logging.getLogger()
		loglevel = logger.getEffectiveLevel()
		logger.setLevel(logging.WARNING)
		ret = f(*args, **kwargs)
		logger.setLevel(loglevel)
		return ret
	return wrapper