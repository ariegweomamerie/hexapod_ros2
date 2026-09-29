import os
from glob import glob
from setuptools import setup

package_name = 'hexapod_worlds'


def tree(source_dir):
    """Install a directory tree (models keep their folder layout so model:// works)."""
    out = []
    for path, _dirs, files in os.walk(source_dir):
        if files:
            out.append((os.path.join('share', package_name, path),
                        [os.path.join(path, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ariegwe',
    maintainer_email='ariegweomamerie@gmail.com',
    description='Simulation worlds for the hexapod robot, built for RGB-D visual SLAM.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_slam_world = hexapod_worlds.generate:main',
            'validate_slam_world = hexapod_worlds.validate:main',
            'inspect_facility_views = hexapod_worlds.inspect_views:main',
            'drive_facility_loop = hexapod_worlds.drive_loop:main',
        ],
    },
)
