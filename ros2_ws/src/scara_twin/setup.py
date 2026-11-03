import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'scara_twin'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laura',
    maintainer_email='laura@todo.todo',
    description='Puente robot real a gemelo digital del SCARA',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge = scara_twin.bridge_node:main',
            'fake_scara = scara_twin.fake_scara:main',
        ],
    },
)
