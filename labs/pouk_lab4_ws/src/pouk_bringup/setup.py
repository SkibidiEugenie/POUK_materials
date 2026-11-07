import os
from glob import glob

from setuptools import setup

package_name = 'pouk_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='M. A. Shereuzhev',
    maintainer_email='shereuzhev@gmail.com',
    description='POUK stand bringup: deployment profiles, launch files, self-checks, clock probe, TF audit',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'check_bridge = pouk_bringup.check_bridge:main',
            'check_coordinator = pouk_bringup.check_coordinator:main',
            'check_moveit = pouk_bringup.check_moveit:main',
            'clock_probe = pouk_bringup.clock_probe:main',
            'tf_audit = pouk_bringup.tf_audit:main',
            'add_obstacle = pouk_bringup.add_obstacle:main',
        ],
    },
)
