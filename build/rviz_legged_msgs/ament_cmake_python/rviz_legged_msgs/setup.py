from setuptools import find_packages
from setuptools import setup

setup(
    name='rviz_legged_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('rviz_legged_msgs', 'rviz_legged_msgs.*')),
)
