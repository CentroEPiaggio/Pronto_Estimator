from setuptools import find_packages
from setuptools import setup

setup(
    name='velocity_command_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('velocity_command_msgs', 'velocity_command_msgs.*')),
)
