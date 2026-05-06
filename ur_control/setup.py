from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ur_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cristian Beltran',
    maintainer_email='cristianbehe@gmail.com',
    description='The ur_control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ft_filter = ur_control.ft_filter:main',
            'controller_examples = ur_control.controller_examples:main',
            'compliance_controller_examples = ur_control.compliance_controller_examples:main',
            'cartesian_compliance_controller_examples = ur_control.cartesian_compliance_controller_examples:main',
            'joint_position_keyboard = ur_control.joint_position_keyboard:main',
            'joint_position_mouse6d = ur_control.joint_position_mouse6d:main',
            'moveit_tutorial = ur_control.moveit_tutorial:main',
            'wrench_republish = ur_control.wrench_republish:main',
            'imu = ur_control.imu:main',
        ],
    },
)
