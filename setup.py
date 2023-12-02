import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_clock_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martiño Crespo Álvarez',
    maintainer_email='marticres@gmail.com',
    description='ROS 2 clock example package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'past_clock_publisher = ros2_clock_examples.past_clock_publisher:main',
            'fake_temperature_sensor_publisher = ros2_clock_examples.fake_temperature_sensor_publisher:main',
            'fake_temperature_sensor_subscriber = ros2_clock_examples.fake_temperature_sensor_subscriber:main',
        ],
    },
)
