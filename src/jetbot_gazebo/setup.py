from setuptools import find_packages, setup

package_name = 'jetbot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['jetbot_gazebo', 'jetbot_gazebo.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models/jetbot', ['models/jetbot/jetbot.sdf']),
        ('share/' + package_name + '/worlds', ['worlds/test_world.sdf']),
        ('share/' + package_name + '/launch', ['launch/jetbot_gazebo.launch.py']),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='mpastor2',
    maintainer_email='mpastor2@student.umb.sk',
    description='JetBot Gazebo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
             'jetbot_node = jetbot_gazebo.jetbot_node:main',
             'path_publisher = jetbot_gazebo.scripts.path_publisher:main',
             'collision_test = jetbot_gazebo.scripts.collision_test:main',
             'obstacle_stop = jetbot_gazebo.scripts.obstacle_stop:main',
             'obstacle_avoid = jetbot_gazebo.scripts.obstacle_avoid:main',
             'run_to_goal = jetbot_gazebo.scripts.run_to_goal:main',
             'follow_wall = jetbot_gazebo.scripts.follow_wall:main',
             'bug0_algoritmus = jetbot_gazebo.scripts.bug0_algoritmus:main',
             'bug1_algoritmus = jetbot_gazebo.scripts.bug1_algoritmus:main',
             'tracking_node = jetbot_gazebo.scripts.tracking_node:main',
        ],
    },
)