from setuptools import find_packages, setup

package_name = 'compass_node'

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
            # ИСПРАВЛЕНО: исходники лежат в папке compass_control/compass_control/
            # (find_packages() находит пакет "compass_control", а НЕ "compass_node"),
            # поэтому entry_point должен ссылаться на compass_control.compass_node,
            # а не на несуществующий модуль compass_node.compass_node.
            # ROS-имя пакета (compass_node, см. package.xml) от этого не меняется.
            'compass_node = compass_control.compass_node:main',
        ],
    },
)
