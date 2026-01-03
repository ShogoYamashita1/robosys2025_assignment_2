from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robosys2025_assignment_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shogo Yamashita',
    maintainer_email='s24c1130pf@s.chibakoudai.jp',
    description='a package for assignment',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = robosys2025_assignment_2.talker:main',
            'listener = robosys2025_assignment_2.listener:main',
            'hand = robosys2025_assignment_2.hand_input:main',
        ],
    },
)
