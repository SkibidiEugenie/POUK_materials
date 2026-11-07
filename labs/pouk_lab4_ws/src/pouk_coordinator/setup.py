from setuptools import setup

package_name = 'pouk_coordinator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='M. A. Shereuzhev',
    maintainer_email='shereuzhev@gmail.com',
    description='POUK stand coordinator: finite state machine behind ROS 2 services',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'coordinator = pouk_coordinator.node:main',
        ],
    },
)
