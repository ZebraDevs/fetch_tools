#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['fetch_tools', 'fetch_tools.commands'],
    package_dir={'': 'src'},
    scripts=['scripts/fetch']
)

setup(**d)
