import os
from pathlib import Path
from setuptools import find_packages, setup

package_name = 'assets_package'

def get_data_files():
    """Generate data_files list including deep_robotics_model directory."""
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Recursively add deep_robotics_model directory
    deep_robotics_path = Path('deep_robotics_model')
    if deep_robotics_path.exists():
        for root, dirs, files in os.walk(deep_robotics_path):
            if files:
                rel_path = Path(root).relative_to('.')
                file_list = [str(Path(root) / f) for f in files]
                dest_dir = f'share/{package_name}/{rel_path}'
                data_files.append((dest_dir, file_list))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
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
        ],
    },
)
