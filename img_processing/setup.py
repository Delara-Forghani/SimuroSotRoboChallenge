#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	packages=['img_processing'],
	scripts=['scripts/vision.py','scripts/subscribe_coordinate.py'],
	packages_dir={'':'src'}
)

setup(**d)