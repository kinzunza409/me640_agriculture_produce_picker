from setuptools import find_packages, setup

package_name = 'husky_hardware_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='inzukyle@gmail.com',
    description='Minimal Husky hardware smoke tests for conservative base velocity commands.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'straight_drive_test = husky_hardware_tests.straight_drive_test:main',
        ],
    },
)
