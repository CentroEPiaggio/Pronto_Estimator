from setuptools import find_packages
from setuptools import setup

setup(
    name='generalized_pose_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('generalized_pose_msgs', 'generalized_pose_msgs.*')),
)
