from setuptools import find_packages
from setuptools import setup

setup(
    name='robot_model',
    version='0.0.0',
    packages=find_packages(
        include=('robot_model', 'robot_model.*')),
)
