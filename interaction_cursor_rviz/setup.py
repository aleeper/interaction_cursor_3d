from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  packages=['interaction_cursor_rviz'],
  scripts=['scripts/Test.py',
           'scripts/HydraTest.py',
           'scripts/PR2HydraTest.py'],
  package_dir={'': 'src'}
)

setup(**d)

