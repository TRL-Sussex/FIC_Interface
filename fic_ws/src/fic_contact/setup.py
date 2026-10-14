from setuptools import setup
from glob import glob

package_name = 'fic_contact'

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
    description='Link-level contact localization from joint torque residuals',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'contact_node = fic_contact.contact_node:main',
            'twin = fic_contact.twin:main',
            'simtouch = fic_contact.simtouch:main',
        ],
    },
)
