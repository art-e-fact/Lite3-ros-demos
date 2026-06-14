from setuptools import find_packages, setup

package_name = 'rail_inspector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name + '/launch', ['launch/rail_target_follow.launch.py']),
        ('share/' + package_name + '/config', ['config/rail_inspector.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='andras@artefacts.com',
    description='Rail detection and rail target following nodes.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rail_inspect_controller = rail_inspector.rail_inspect_controller:main',
            'rail_detector_node = rail_inspector.rail_detector_node:main',
            'rail_target_follower_node = rail_inspector.rail_target_follower_node:main',
            'relay_node = rail_inspector.relay_node:main'
        ],
    },
)
