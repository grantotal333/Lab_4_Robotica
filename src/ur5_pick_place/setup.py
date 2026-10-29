
from setuptools import find_packages, setup

package_name = 'ur5_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Diego',
    maintainer_email='student@example.com',
    description='Rutina Pick and Place para UR5 con gripper',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pick_place_node = ur5_pick_place.pick_place_node:main',
        ],
    },
)
