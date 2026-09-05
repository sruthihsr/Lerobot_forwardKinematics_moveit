#!/usr/bin/env python3
"""
Setup script for the SO-101 Forward Kinematics & MoveIt Workspace
"""

from setuptools import setup, find_packages

setup(
    name='so101_forward_kinematics_moveit',
    version='1.0.0',
    description='SO-101 arm: kinematics, MoveIt planning, and real-hardware control',
    author='Sruthi Mohan',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    python_requires='>=3.10',
    install_requires=[
        'numpy>=1.24.0',
        'scipy>=1.10.0',
        'matplotlib>=3.7.0',
    ],
    extras_require={
        'mujoco': ['mujoco>=3.0.0'],
        'ros2': [],  # ROS2 packages installed via rosdep
        'lerobot': ['lerobot[kinematics]'],
        'all': [
            'mujoco>=3.0.0',
            'opencv-python>=4.8.0',
        ],
    },
    entry_points={
        'console_scripts': [
            # Console scripts for labs will be added here as the course progresses
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Education',
        'Topic :: Scientific/Engineering :: Robotics',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
    ],
)
