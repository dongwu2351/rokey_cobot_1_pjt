from setuptools import find_packages, setup

package_name = 'palpa_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='palpa_team',
    maintainer_email='team@palpa.local',
    description='PALPA 로봇 검사/제어 노드 모음',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_controller_node = palpa_control.main_controller_node:main',
            'robot_controller_node = palpa_control.robot_controller_node:main',
            'robot_controller_stub_node = palpa_control.robot_controller_stub_node:main',
            'exception_handler_node = palpa_control.exception_handler_node:main',
        ],
    },
)
