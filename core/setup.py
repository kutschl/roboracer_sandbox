from setuptools import setup
import os
from glob import glob

package_name = 'core'

# -------------------------------------------------------------------------
# Car configs
# -------------------------------------------------------------------------
car_versions = [
    'alonso', 
    'leclerc', 
    'vettel', 
    'rosberg'
]

racecar_config_subdirs = []

def add_config_tree(base_dir, share_prefix):
    for root, dirs, files in os.walk(base_dir):
        if not files:
            continue
        rel_path = os.path.relpath(root, base_dir)
        if rel_path == '.':
            rel_path = ''
        target = os.path.join(share_prefix, rel_path)
        file_list = [os.path.join(root, f) for f in files]
        racecar_config_subdirs.append((target, file_list))

for car in car_versions:
    base       = os.path.join('config', car)
    share_pref = os.path.join('share', package_name, 'config', car)
    add_config_tree(base, share_pref)
    
    
# -------------------------------------------------------------------------
# Map files
# -------------------------------------------------------------------------
map_subdirs = []

for map_dir in os.listdir('maps'):
    map_subdirs.append((
        os.path.join('share', package_name, 'maps', map_dir),
        glob(f'maps/{map_dir}/*.png', recursive=True)
    ))
    map_subdirs.append((
        os.path.join('share', package_name, 'maps', map_dir),
        glob(f'maps/{map_dir}/*.yaml', recursive=True)
    ))

    map_subdirs.append((
        os.path.join('share', package_name, 'maps', map_dir, "plans"),
        glob(f'maps/{map_dir}/plans/*.csv', recursive=True)
    ))

    map_subdirs.append((
        os.path.join('share', package_name, 'maps', map_dir, "plans"),
        glob(f'maps/{map_dir}/plans/*.json', recursive=True)
    ))


# -------------------------------------------------------------------------
# Package setup
# -------------------------------------------------------------------------
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.*.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch', 'subsystems'), glob(os.path.join('launch', 'subsystems', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        *racecar_config_subdirs,
        *map_subdirs,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lukas Kutsch',
    maintainer_email='lukas.kutsch@uni-bonn.de',
    description='The package to rule them all: Launch files and Configs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        # 'console_scripts': [
        #     'global_parameter_node = core.global_parameter_node:main'
        # ]
    }
)
