from setuptools import find_packages, setup
from os.path import join
from glob import glob

package_name = 'devol_demo_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob(join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arc-lsi-dev',
    maintainer_email='arc-lsi-dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'devol_demo = devol_demo_py.devol_demo:main',
            'devol_drive_demo = devol_demo_py.devol_drive_demo:main',
            'dual_devol_demo = devol_demo_py.dual_devol_demo:main',
            'world_state_publisher = devol_demo_py.world_state_publisher:main',
            'collision_checker = devol_demo_py.collision_checker:main'
        ],
    },
)
