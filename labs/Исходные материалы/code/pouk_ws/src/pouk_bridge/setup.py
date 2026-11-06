from setuptools import setup

package_name = 'pouk_bridge'

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
    description='POUK stand bridge: ROS 2 <-> stand protocol v2, FollowJointTrajectory executor',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'bridge = pouk_bridge.bridge_node:main',
        ],
    },
)
