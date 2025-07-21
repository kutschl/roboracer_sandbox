from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'global_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name]+find_packages(),
    data_files=[
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Denninger',
    maintainer_email='l_denninger@uni-bonn.de',
    description='The global planner package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
