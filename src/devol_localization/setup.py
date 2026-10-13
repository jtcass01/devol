from glob import glob
from setuptools import find_packages, setup

package_name = 'devol_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakeadelic',
    maintainer_email='jacobtaylorcassady@outlook.com',
    description='Map-based localization for the devol mobile manipulator (EKF and particle filter).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pf_node = devol_localization.pf_localization_node:main',
            'ekf_node = devol_localization.ekf_localization_node:main',
            'dr_node = devol_localization.dead_reckoning_node:main',
            'ground_truth_node = devol_localization.ground_truth_node:main',
            'viz_node = devol_localization.localization_vis_node:main',
            'metrics_node = devol_localization.metrics_vis_node:main',
        ],
    },
)
