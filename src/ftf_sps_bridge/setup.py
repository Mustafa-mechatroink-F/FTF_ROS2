from setuptools import setup

package_name = 'ftf_sps_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
    ('share/ftf_sps_bridge/launch', ['launch/sps_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mustafa',
    maintainer_email='mustafa@example.com',
    description='Bridge zwischen Beckhoff SPS (ADS) und ROS2 (Odometrie + cmd_vel)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sps_bridge_node= ftf_sps_bridge.sps_bridge_node:main',
        ],
    },
)
