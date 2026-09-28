import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bel_amr_ros'

setup(
    name=package_name,
    version='0.0.1',

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
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='yazhini',
    maintainer_email='yazhini@example.com',

    description='ROS 2 decentralized controller package for BEL AMR fleet',

    license='Apache-2.0',

    entry_points={
    'console_scripts': [
        'robot_controller = bel_amr_ros.robot_controller:main',
        'p2p_agent = bel_amr_ros.p2p_agent:main',
        'webots_navigation = bel_amr_ros.webots_nodes:navigation_main',
        'webots_p2p = bel_amr_ros.webots_nodes:peer_main',
        'federated_agent = bel_amr_ros.federated_agent:main',
    ],
},
)