from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['localization_common'],
    #scripts=['scripts/utilities.py'],
    package_dir={'': 'scripts'}
)

setup(**d)
