from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ur_pykdl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cristian Beltran',
    maintainer_email='beltran@hlab.sys.es.osaka-u.ac.jp',
    description='Simple implementation of PyKDL, kdl_parser_py',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur_kinematics = ur_pykdl.ur_kinematics:main',
            'display_urdf = ur_pykdl.display_urdf:main',
        ],
    },
)
