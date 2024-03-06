from setuptools import find_packages
from setuptools import setup

setup(
    name='rviz_legged_plugins',
    version='0.0.0',
    packages=find_packages(
        include=('rviz_legged_plugins', 'rviz_legged_plugins.*')),
)
