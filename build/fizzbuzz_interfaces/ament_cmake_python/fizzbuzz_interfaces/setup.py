from setuptools import find_packages
from setuptools import setup

setup(
    name='fizzbuzz_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('fizzbuzz_interfaces', 'fizzbuzz_interfaces.*')),
)
