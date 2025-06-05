import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cluster_finder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.rviz')),

        # These lines make sure the launch files are installed
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan M',
    maintainer_email='martnat8@oregonstate.edu',
    description='Counts and publishes the number of clusters in a PointCloud2 scan',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cluster_finder = cluster_finder.cluster_finder:main'
        ],
    },
)
