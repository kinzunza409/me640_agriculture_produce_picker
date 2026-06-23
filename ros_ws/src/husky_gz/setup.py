from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'husky_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        # installs launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        # installs world files both under the package and at the share root
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf'),
        ),
        (
            'share',
            glob('worlds/*.sdf'),
        ),
        # installs the default robot config
        (
            os.path.join('share', package_name, 'config', 'husky'),
            glob('config/husky/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='inzukyle@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_pose_logger = husky_gz.pid_pose_logger:main',
            'pid_bag_to_csv = husky_gz.pid_bag_to_csv:main',
        ],
    },
)
