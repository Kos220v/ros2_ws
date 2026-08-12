from setuptools import setup
import os
from glob import glob

package_name = 'kolesa_control'

# Собираем launch‑файлы, если они существуют
launch_files = glob('launch/*.launch.py')

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Добавляем launch‑файлы только если они найдены
if launch_files:
    data_files.append(
        (os.path.join('share', package_name, 'launch'), launch_files)
    )

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=[
        'setuptools',
        'pyserial',
        'rclpy',
    ],
    zip_safe=True,
    maintainer='Anton',
    maintainer_email='anton@example.com',
    description='Дифференциальное управление гусеничным роботом через два FS75100 (VESC) по UART.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kolesa_control = kolesa_control.kolesa_control_node:main',
        ],
    },
)
