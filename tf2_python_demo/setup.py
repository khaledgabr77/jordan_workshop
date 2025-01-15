from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tf2_python_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Khaled Gabr',
    maintainer_email='khaledgabr77@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_broadcaster = tf2_python_demo.static_broadcaster:main',
            'tf_listener = tf2_python_demo.tf_listener:main',
            'dynamic_transform_broadcaster = tf2_python_demo.dynamic_transform_broadcaster:main',
            'turtle_tf2_pose = tf2_python_demo.turtle_tf2_pose:main',
            'turtle_tf2_follower = tf2_python_demo.turtle_tf2_follower:main',
        ],
    },
)

