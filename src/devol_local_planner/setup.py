from glob import glob
from setuptools import find_packages, setup

package_name = 'devol_local_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakeadelic',
    maintainer_email='jacobtaylorcassady@outlook.com',
    description='Local motion planning for the devol mobile base: RRT* / A* planners and the PID path follower',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'diffdrive_pid = devol_local_planner.diffdrive_pid:main',
            'agent_motion_planner = devol_local_planner.agent_motion_planner:main',
            'rrt_motion_planner = devol_local_planner.rrt_motion_planner:main',
            'map_padder = devol_local_planner.map_padder:main',
        ],
    },
)
