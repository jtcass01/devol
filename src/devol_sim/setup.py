from glob import glob
from setuptools import find_packages, setup

package_name = 'devol_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakeadelic',
    maintainer_email='jacobtaylorcassady@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diffdrive_pid = devol_sim.diffdrive_pid:main',
            'agent_motion_planner = devol_sim.agent_motion_planner:main',
            'map_publisher = devol_sim.map_publisher:main',
            'goal_points_publisher = devol_sim.goal_points_publisher:main',
        ],
    },
)
