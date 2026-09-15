from setuptools import find_packages, setup

package_name = 'imu_stm32_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu.launch.py']),
        ('share/' + package_name + '/config', ['config/imu_params.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Мост STM32 IMU (UART) -> ROS 2.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = imu_stm32_bridge.bridge_node:main',
        ],
    },
)
