from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['robot_calibration'],
    scripts=['scripts/calibrate_from_aruco'],
    package_dir={'': 'src'}
)

setup(**d)