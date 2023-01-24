from setuptools import setup

package_name = 'camera_subscriber'

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
    maintainer='Maciej',
    maintainer_email='maciej.p.krupka@student.put.poznan.pl',
    description='Projekt zaliczeniowy przedmiot NIOSR',
    license='TODO: License declaration',
    tests_require=['pytest','unittest', 'math'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_subscriber.camera_node:main',
            'robot_control = camera_subscriber.robot_control:main',
            'usb_imu = camera_subscriber.usb_imu:main'
        ],
    },
)
