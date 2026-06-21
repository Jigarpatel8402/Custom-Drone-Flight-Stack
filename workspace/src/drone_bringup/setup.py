from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'drone_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/drone_bringup'],
        ),
        (
            'share/drone_bringup', 
            ['package.xml'],
        ),
        (
            os.path.join(
                'share',
                'drone_bringup',
                'launch'
            ),
            glob('launch/*.py'),
        ),
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
        ],
    },
)
