from setuptools import find_packages
from setuptools import setup

setup(
    name='lab2_iic2685',
    version='0.0.0',
    packages=find_packages(
        include=('lab2_iic2685', 'lab2_iic2685.*')),
)
