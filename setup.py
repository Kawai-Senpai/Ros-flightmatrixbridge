from setuptools import setup

package_name = 'flightmatrix_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ranit Bhowmick',
    maintainer_email='bhowmickranitking@duck.com',
    description='ROS2 wrapper for the FlightMatrix API',
    license='MIT',
    entry_points={
        'console_scripts': [
            'flightmatrix_publisher = flightmatrix_ros2.flightmatrix_publisher:main',
            'drone_controller = flightmatrix_ros2.drone_controller:main',
            'data_recorder = flightmatrix_ros2.data_recorder:main',
        ],
    },
)
