from setuptools import find_packages, setup

package_name = 'f4_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/f4_bridge.launch.py']),
        ('share/' + package_name + '/config', ['config/f4_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Maintainer',
    maintainer_email='maintainer@example.com',
    description='Safe ROS2 integration facade for STM32F411 USB CDC winch/HMI.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'f4_bridge_node = f4_bridge.bridge_node:main',
        ],
    },
)
