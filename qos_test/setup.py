from setuptools import find_packages, setup

package_name = 'qos_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khaled',
    maintainer_email='khaledgabr77@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qos_publisher = qos_test.qos_publisher:main',
            'qos_subscriber = qos_test.qos_subscriber:main',
            'qos_reliability_publisher = qos_test.qos_reliability_publisher:main',
            'qos_reliability_subscriber = qos_test.qos_reliability_subscriber:main',

        ],
    },
)
