from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'my_fsm_bumpgo_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ojigrande',
    maintainer_email='ojigrande@gmail.com',
    description='My experiments with FSM and ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bump_go_main = my_fsm_bumpgo_py.bump_go_main:main'
        ],
    },
)
