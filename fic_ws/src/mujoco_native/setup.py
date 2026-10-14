from setuptools import setup
from glob import glob

package_name = 'mujoco_native'

setup(
    name=package_name,
    version='4.5.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adnan',
    maintainer_email='at717@sussex.ac.uk',
    description='Native CPU MuJoCo backend via the official mujoco Python wheel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation = mujoco_native.simulation_node:main',
        ],
    },
)
