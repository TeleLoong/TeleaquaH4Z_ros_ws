from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teleh4z_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TeleH4Z Dev',
    maintainer_email='dev@teleh4z.local',
    description='TeleH4Z mode switch orchestrator with arm deploy/retract',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mode_manager = teleh4z_manager.mode_manager:main',
        ],
    },
)
