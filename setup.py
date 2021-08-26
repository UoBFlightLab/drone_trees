from setuptools import setup

setup(name='drone_trees',
      version='1.0.1',
      description='Behaviour trees for drone control',
      url='https://github.com/UoBFlightLab/drone_trees',
      author='UoBFlightLab',
      author_email='flightlab@example.com',
      license='MIT',
      packages=['drone_trees'],
      install_requires=['dronekit','dronekit_sitl','py_trees','wheel'],
      zip_safe=False)
