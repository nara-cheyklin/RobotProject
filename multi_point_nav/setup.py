from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_point_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
	],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jojoemen',
    maintainer_email='jojoemen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'multi_point_nav_node = multi_point_nav.multi_point_nav_node:main',
            'hand_gesture_node = multi_point_nav.hand_gesture_node:main',
     ],
    },
)
