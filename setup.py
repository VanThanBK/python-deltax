import os
import sys
from setuptools import setup, find_packages

setup(
    name="deltax",
    author="Than Nguyen",
    author_email="jonyvanthan@gmail.com",
    description="Python library to control an DeltaX robot",
    long_description_content_type="text/markdown",
    url="https://github.com/VanThanBK/python-deltax",
    project_urls={
        "Bug Tracker": "https://github.com/VanThanBK/python-deltax",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    install_requires=[
        "PySide6>=6.0.0",
    ],
    python_requires=">=3.6",
)