from setuptools import find_packages, setup

package_name = 'drone_control'

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
    maintainer='jigar',
    maintainer_email='jigarpatel8402@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'attitude_control = drone_control.attitude_controller:main',
            'position_control = drone_control.position_controller:main',
            'altitude_velocity_control = drone_control.altitude_velocity_controller:main',
            'xy_velocity_control = drone_control.xy_velocity_controller:main',
        ],
    },
)
