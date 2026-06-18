from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'husky_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # installs launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # installs the default robot config
        (os.path.join('share', package_name, 'config', 'husky'),
            glob('config/husky/*.yaml')),
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
        ],
    },
)
