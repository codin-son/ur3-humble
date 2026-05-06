from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ur_gripper_gazebo'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ] + [
        (os.path.join('share', package_name, root),
         [os.path.join(root, f) for f in files])
        for root, dirs, files in os.walk('models')
        if files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cristian Beltran',
    maintainer_email='beltran@hlab.sys.es.osaka-u.ac.jp',
    description='Gazebo Classic simulation for UR robot with Robotiq gripper',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_to_tf = ur_gazebo.gazebo_to_tf:main',
            'spawner = ur_gazebo.spawner:main',
            'world_publisher = ur_gazebo.world_publisher:main',
        ],
    },
)
