from setuptools import find_packages, setup

package_name = 'jetbot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['jetbot_description', 'jetbot_description.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models/urdf', ['models/urdf/jetbot.urdf']),
        ('share/' + package_name + '/launch', ['launch/jetbot_rviz.launch.py']),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='mpastor2',
    maintainer_email='mpastor2@student.umb.sk',
    description='JetBot Description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'jetbot_node = jetbot_description.jetbot_node:main',
            'path_publisher = jetbot_description.scripts.path_publisher:main',
            'cmd_vel_to_odom = jetbot_description.scripts.cmd_vel_to_odom:main'
        ],
    },
)
