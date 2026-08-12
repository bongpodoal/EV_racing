from setuptools import find_packages, setup

package_name = 'ev_arduino_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bongpodoal',
    maintainer_email='kbc6710@gmail.com',
    description='VehicleCommand를 시리얼로 아두이노 후륜구동 CAN 브릿지에 전달',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = ev_arduino_bridge.arduino_bridge_node:main',
        ],
    },
)
