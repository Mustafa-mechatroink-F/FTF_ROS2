from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ftf_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'images'), glob('resource/images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mustafa',
    description='FTF Steuerung und GUI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ftf_drive = ftf_control.ftf_sps_drive_node:main',
            'dock_laden = ftf_control.docking_node:main',
            'unload_return = ftf_control.unload_return_node:main',
            'ftf_gui = ftf_control.ftf_gui:main',
        ],
    },
)