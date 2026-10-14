from setuptools import setup, find_packages

setup(
    name="fic_api",
    version="0.1.0",
    description="Python API for FIC-controlled robots (Franka FR3, Kinova Gen3, IIWA)",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="FIC Studio",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=[],   # rclpy comes from the ROS2 environment
    extras_require={
        "scipy": ["scipy"],  # optional, for Euler→quaternion in move_cartesian
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
)
