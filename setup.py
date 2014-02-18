#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['nao_camera'],
    package_dir={'': 'python'},
    scripts=['python/nodes/nao_camera_node.py']
)

setup(**d)
