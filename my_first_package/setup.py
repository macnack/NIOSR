from setuptools import setup

package_name = 'my_first_package'

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
    license='MIT',
    tests_require=['pytest','unittest', 'math'],
    entry_points={
        'console_scripts': [
            'my_first_node = my_first_package.my_first_node:main',
            'camera_node = my_first_package.camera_node:main',
            'robot_control = my_first_package.robot_control:main',
            'usb_imu = my_first_package.usb_imu:main'
        ],
    },
)
