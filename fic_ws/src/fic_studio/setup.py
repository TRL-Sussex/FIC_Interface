from setuptools import setup, find_packages

package_name = 'fic_studio'

setup(
    name=package_name,
    version='6.0.1',
    packages=find_packages(),
    # The web remote's page. Without this a copy-install (macOS) ships only
    # the .py files and the server has nothing to serve.
    package_data={'fic_studio.web_remote': ['static/*', 'ui/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='FIC Control Studio - PyQt5 GUI for robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fic_studio_pro = fic_studio.entry_pro:main',
            'fic_dev = fic_studio.entry_dev:main',
        ],
    },
)
