from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['task_planner'],
    package_dir={'': 'src'},
    install_requires=[
        'pydantic>=2.0.0',
    ]
)

setup(**d)