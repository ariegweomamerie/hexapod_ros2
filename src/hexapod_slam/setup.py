from glob import glob

from setuptools import find_packages, setup

package_name = 'hexapod_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ap Omamerie Ariegwe',
    maintainer_email='ariegweomamerie@gmail.com',
    description='RGB-D visual odometry and SLAM for the hexapod robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_odometry_experiment = hexapod_slam.evaluate_odometry:main',
            'reset_run = hexapod_slam.reset_run:main',
            'score_bag = hexapod_slam.score_bag:main',
            'inspect_failure = hexapod_slam.inspect_failure:main',
        ],
    },
)
