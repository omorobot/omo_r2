import os # add
from glob import glob # add
from setuptools import find_packages, setup

package_name = 'omo_r2_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')), # add
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*')), # add
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')), # add
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')), # add
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr.K',
    maintainer_email='t.shaped.person@gmail.com',
    description='3D models of simulation and visualization for omo_r2',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
