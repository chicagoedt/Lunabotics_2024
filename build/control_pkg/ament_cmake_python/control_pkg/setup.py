from setuptools import find_packages
from setuptools import setup

setup(
    name='control_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('control_pkg', 'control_pkg.*')),
)
