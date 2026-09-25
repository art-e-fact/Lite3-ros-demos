from setuptools import find_packages, setup

package_name = 'nav2_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/nav2_route.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_lite3.yaml']),
        ('share/' + package_name + '/maps', ['maps/depot.yaml', 'maps/depot.pgm', 'maps/sandbox.yaml', 'maps/sandbox.pgm']),
        ('share/' + package_name + '/routes', ['routes/depot.yaml', 'routes/sandbox.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tomo Norman',
    maintainer_email='tomo@artefacts.com',
    description="Drive a recorded route with Nav2 on a quadruped in a world extruded from the route's map.",
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
