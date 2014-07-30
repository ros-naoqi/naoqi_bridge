#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['nao_sensors'],
    package_dir={'': 'src'},
    scripts=['nodes/camera.py', 'nodes/octomap.py']
)

setup(**d)
