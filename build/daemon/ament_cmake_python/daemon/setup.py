from setuptools import find_packages
from setuptools import setup

setup(
    name='daemon',
    version='0.1.0',
    packages=find_packages(
        include=('daemon', 'daemon.*')),
)
