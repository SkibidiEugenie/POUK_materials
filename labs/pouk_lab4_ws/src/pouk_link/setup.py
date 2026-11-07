from setuptools import setup

package_name = 'pouk_link'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='M. A. Shereuzhev',
    maintainer_email='shereuzhev@gmail.com',
    description='POUK stand protocol v2, link, kinematics, virtual stand',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sim_mcu = pouk_link.sim_mcu:main',
            'cli = pouk_link.cli:main',
            'analyze = pouk_link.analyze:main',
        ],
    },
)
