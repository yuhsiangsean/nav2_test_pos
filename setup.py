from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'nav2_test_pos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='a0929370607@gmail.com',
    description=(
        'px4_nav2_bridge 的位置控制版：controller_server 換成自己實作的 '
        'FollowPath action server，直接送位置 setpoint 給 PX4。'
    ),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_controller = nav2_test_pos.position_controller:main',
            'px4_odom_tf = nav2_test_pos.px4_odom_tf:main',
        ],
    },
)
