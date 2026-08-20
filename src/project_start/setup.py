import os
from glob import glob
from setuptools import setup

package_name = 'project_start'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # ИСПРАВЛЕНО: раньше папка config/ вообще не устанавливалась colcon'ом,
        # поэтому get_package_share_directory('project_start') + 'config'
        # (используется в start.launch.py) не находил бы ни один YAML-файл
        # после `colcon build` (без --symlink-install). Теперь и config/,
        # и params/ (оставлено для совместимости) копируются в share/.
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@example.com',
    description='Your package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ИСПРАВЛЕНО: 'your_node' — заготовка от `ros2 pkg create`, файла
            # project_start/your_node.py не существует. Этот пакет содержит
            # только launch-файлы и конфиги, исполняемых узлов у него нет,
            # поэтому entry_points оставлен пустым.
        ],
    },
)
