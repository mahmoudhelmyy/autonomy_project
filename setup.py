from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomy_project'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='client',
    maintainer_email='client@example.com',
    description='Autonomous control package for the Moebius Challenge.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = autonomy_project.navigation_node:main',
            'obstacle_detector_node = autonomy_project.obstacle_detector_node:main',
        ],
    },
)