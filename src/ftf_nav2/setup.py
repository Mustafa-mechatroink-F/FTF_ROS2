from setuptools import setup
import os
from glob import glob

package_name = 'ftf_nav2'

setup(
    name=package_name,
    # ... andere Felder ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Diese Zeilen stellen sicher, dass die Dateien kopiert werden:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    # ...
)