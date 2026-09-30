from glob import glob

from setuptools import setup

package_name = 'terraslam_solucion_dock_lidar'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Echevarria',
    maintainer_email='patrick.echevarria.d@gmail.com',
    description='Docking del Create 3 usando solo el LiDAR.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dock_lidar = terraslam_solucion_dock_lidar.dock_lidar:main',
        ],
    },
)
