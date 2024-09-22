import os # add
from glob import glob # add
from setuptools import setup

package_name = 'omo_r2_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')), # add
        (os.path.join('share', package_name, 'param'), glob('param/*')), # add
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr.K',
    maintainer_email='t.shaped.person@gmail.com',
    description='Driver for omo_r2',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control = omo_r2_robot.robot_control:main'
        ],
    },
)
