from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['droplet_underwater_assembly_libs'],
    package_dir={'': 'src'}
)

setup(**d)
