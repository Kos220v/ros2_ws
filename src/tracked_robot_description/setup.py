import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tracked_robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Регистрация пакета в ROS2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch файлы — все *.launch.py из папки launch/
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),

        # URDF/Xacro файлы — все файлы из папки urdf/
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),

        # RViz конфигурации — все *.rviz файлы
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Описание гусеничного робота для RViz2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
