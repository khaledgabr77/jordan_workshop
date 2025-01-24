from setuptools import find_packages, setup

package_name = 'py_service_pkg'

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
            'py_service_server = py_service_pkg.service_server:main',
            'py_service_client = py_service_pkg.service_client:main'
        ],
    },
)
