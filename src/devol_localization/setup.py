from setuptools import find_packages, setup
from os.path import join
from glob import glob

package_name = 'devol_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob(join('launch', '*launch.[pxy][yma]*'))),
        (join('share', package_name, 'config'), glob(join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jtcass01',
    maintainer_email='jacobtaylorcassady@outlook.com',
    description='Localization estimators (particle filter, EKF) for the devol mobile manipulator.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pf_localization = devol_localization.pf_localization_node:main',
        ],
    },
)
