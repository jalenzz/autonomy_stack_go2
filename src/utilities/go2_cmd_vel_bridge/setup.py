import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'go2_cmd_vel_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autonomy_stack_go2 maintainers',
    maintainer_email='maintainer@todo.todo',
    description='Bridge /cmd_vel to Go2 Sport Move with wireless remote takeover.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = go2_cmd_vel_bridge.cmd_vel_bridge:main',
        ],
    },
)
