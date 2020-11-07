import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="dymoesco",
    version="0.0.1",
    author="Samuel Laferriere",
    author_email="samlaf92@gmail.com",
    description="Dynamics Modeling, Estimation and Control",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/samlaf/dymoesco",
    packages=setuptools.find_packages(), # https://packaging.python.org/guides/packaging-namespace-packages/
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
