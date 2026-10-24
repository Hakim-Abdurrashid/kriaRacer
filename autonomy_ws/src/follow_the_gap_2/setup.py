from setuptools import setup
import os
from glob import glob

package_name = 'follow_the_gap_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Israel Charles',
    maintainer_email='israelcharlesic10@gmail.com',
    description='Follow the Gap reactive planner',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follow_the_gap_2_node = follow_the_gap_2.follow_the_gap_2_node:main',
        ],
    },
)
