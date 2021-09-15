import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="deltax",
    version="0.0.1.1",
    author="Than Nguyen",
    author_email="jonyvanthan@gmail.com",
    description="Python library to control an DeltaX robot",
    long_description=long_description,
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
    packages=setuptools.find_packages(where="src"),
    python_requires=">=3.6",
)