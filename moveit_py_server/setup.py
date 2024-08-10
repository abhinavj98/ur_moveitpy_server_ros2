from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'moveit_py_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav Jain',
    maintainer_email='abhinav98jain@gmail.com',
    description='Use MoveIt2 with Python API and real UR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'moveit_py_server = moveit_py_server.moveit_py_server:main',
            'moveit_py_server_client = moveit_py_server.moveit_py_server_client:main',
        ],
    },
)
