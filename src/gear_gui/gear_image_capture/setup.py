#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup, find_packages

package_info = generate_distutils_setup(
    packages=find_packages('src'),
    package_dir={'': 'src'}
)

setup(**package_info)