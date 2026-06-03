from pathlib import Path

from setuptools import find_packages, setup


package_name = 'simulation_package'
package_root = Path(__file__).parent.resolve()


def _data_files(source_dir: str, install_dir: str) -> list[tuple[str, list[str]]]:
    source_root = package_root / source_dir
    if not source_root.exists():
        return []

    collected: list[tuple[str, list[str]]] = []
    for path in sorted(source_root.rglob('*')):
        if not path.is_file():
            continue
        relative_parent = path.parent.relative_to(source_root)
        destination = Path(install_dir) / relative_parent
        collected.append((str(destination), [str(path.relative_to(package_root))]))
    return collected

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'simulation_package.sensors': ['*.npy'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + _data_files('launch', 'share/' + package_name + '/launch') + _data_files('assets', 'share/' + package_name + '/assets') + _data_files('config', 'share/' + package_name + '/config'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azazdeaz',
    maintainer_email='azazdeaz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simulation_node = simulation_package.simulation_node:main',
            'start_simulation = simulation_package.start_simulation:main',
            'mujoco_simulation_ros2 = simulation_package.mujoco_simulation_ros2:main',
            'newton_simulation_ros2 = simulation_package.newton.newton_simulation_ros2:main',
            'auto_waypoint_navigator = simulation_package.auto_waypoint_navigator:main',
        ],
    },
)
