## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(packages=['roscv_modules', 'local_iq_gnc'], package_dir={'': 'src'})
# setup_args = generate_distutils_setup(packages=find_packages(), package_dir={'': 'src'})

setup(**setup_args)