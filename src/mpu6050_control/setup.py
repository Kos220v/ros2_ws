from setuptools import find_packages, setup

package_name = 'mpu6050_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ИСПРАВЛЕНО: исходники лежат в mpu6050_control/mpu6050_control/,
            # find_packages() находит пакет "mpu6050_control", а не "mpu6050_node".
            # Раньше здесь было 'mpu6050_node.mpu6050_node:main' — несуществующий
            # модуль, из-за чего `ros2 run mpu6050_node mpu6050_node` падал с
            # ModuleNotFoundError. ROS-имя пакета (mpu6050_node) не меняется.
            'mpu6050_node = mpu6050_control.mpu6050_node:main',
        ],
    },
)
