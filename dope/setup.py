#!/usr/bin/env python
import os
from setuptools import find_packages
from setuptools import setup
from glob import glob

package_name = 'dope'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'weights'), glob(os.path.join('weights', '*.pth'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Jonatan Gines',
    author_email='jonatan.gines@urjc.es',
    maintainer='Jonatan Gines',
    maintainer_email='jonatan.gines@urjc.es',
    keywords=['ROS2', 'DOPE', 'NVIDIA'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'The DOPE package'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dope = dope.dope_node:main'
        ],
    },
)
