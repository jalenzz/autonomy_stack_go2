import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'fast_lio_vehicle_adapter'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autonomy_stack_go2 maintainers',
    maintainer_email='maintainer@todo.todo',
    description=(
        'Convert Fast-LIO MID360 IMU odometry to the Go2 vehicle '
        'reference point.'
    ),
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_odometry_adapter = '
            'fast_lio_vehicle_adapter.vehicle_odometry_adapter:main',
        ],
    },
)
