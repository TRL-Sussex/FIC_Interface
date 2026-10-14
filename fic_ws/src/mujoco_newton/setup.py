from setuptools import setup
from glob import glob

package_name = 'mujoco_newton'

setup(
    name=package_name,
    version='4.1.0',
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
    description='GPU-accelerated MuJoCo backend via NVIDIA Newton/Warp',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation = mujoco_newton.simulation_node:main',
        ],
    },
)
