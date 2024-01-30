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
)