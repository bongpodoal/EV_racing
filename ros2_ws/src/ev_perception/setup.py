from setuptools import find_packages, setup

package_name = 'ev_perception'

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
    description='라이다/카메라 기반 전방 장애물 인지',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector_node = ev_perception.obstacle_detector_node:main',
            'cone_detector_node = ev_perception.cone_detector_node:main',
        ],
    },
)
