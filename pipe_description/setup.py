import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pipe_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro") + glob("urdf/*.gazebo") + glob("urdf/*.trans")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.stl")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='biorobotics',
    maintainer_email='varunpat789@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
