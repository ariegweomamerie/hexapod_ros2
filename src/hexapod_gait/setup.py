import os
from glob import glob
from setuptools import setup

package_name = 'hexapod_gait'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ariegwe',
    maintainer_email='ariegweomamerie@gmail.com',
    description='Tripod walking gait and per-leg kinematics for the hexapod robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_node = hexapod_gait.gait_node:main',
        ],
    },
)
