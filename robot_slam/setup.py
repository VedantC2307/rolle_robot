from setuptools import find_packages, setup

package_name = 'robot_slam'

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
    maintainer='vedant',
    maintainer_email='vedantchoudhary16@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node = robot_slam.slam_node:main',
            'base64_camera_data_node = robot_slam.camera_data_node:main',
            'robot_speech_node = robot_slam.robot_speech_node:main',
        ],
    },
)
