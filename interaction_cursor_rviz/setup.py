from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  packages=['interaction_cursor_rviz'],
  scripts=['scripts/interaction_cursor_simple_test.py'],
  package_dir={'': 'src'}
)

setup(**d)

