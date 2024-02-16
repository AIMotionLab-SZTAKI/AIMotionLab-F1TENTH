from setuptools import setup, find_packages

setup(
    name='aimotion_f1tenth_utils',
    version='1.0.0',
    description='Comminucation and scriptign layer for the F1TENTH fleet manager',
    author='K. Floch',
    author_email='flochkristof@sztaki.hu',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'scipy',
        'matplotlib',
    ],
      entry_points = {
        'console_scripts': ['trajectory_creator=aimotion_f1tenth_utils.traj_creator:main'],
    },
)