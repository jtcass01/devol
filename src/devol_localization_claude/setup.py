from glob import glob
from setuptools import find_packages, setup

package_name = 'devol_localization_claude'

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
            'ekf_localization = devol_localization_claude.ekf_localization:main',
            'pf_localization = devol_localization_claude.pf_localization_node:main',
            'noise_injector = devol_localization_claude.noise_injector:main',
            'ground_truth_tf = devol_localization_claude.ground_truth_tf:main',
            'kidnapper = devol_localization_claude.kidnapper:main',
            'localization_evaluator = devol_localization_claude.localization_evaluator:main',
            'localization_viz = devol_localization_claude.localization_viz:main',
            'posterior_snapshot = devol_localization_claude.posterior_snapshot:main',
            'localization_study = devol_localization_claude.localization_study:main',
            'hybrid_supervisor = devol_localization_claude.hybrid_supervisor:main',
        ],
    },
)
