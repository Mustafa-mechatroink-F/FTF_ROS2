from setuptools import find_packages, setup

package_name = 'ftf_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
     data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mustafa',
    maintainer_email='ftf@todo.todo',
    description='FTF Steuerung: SPS Drive, Hub und Station integration.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ftf_drive = ftf_control.ftf_sps_drive_node:main',
            'ftf_hub = ftf_control.ftf_hub_node:main',
            'ftf_station = ftf_control.ftf_station_node:main',
        ],
    },
)
