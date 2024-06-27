
from setuptools import setup, find_packages



setup(
    name='aimotion_f1tenth_utils',
    version='1.0.0',
    description='Communication and scripting layer for the F1TENTH fleet manager',
    packages=find_packages(),
    install_requires=[
        'numpy==1.24.4',
        'sphinx_markdown_builder',
        'recommonmark',
        'myst_parser',
        'scipy',
        'matplotlib',
        'pygame',
        'pyyaml',
        'paramiko',
        'motioncapture==1.0a1; python_version<"3.10" and platform_system != "Darwin"',
        'motioncapture==1.0a2; python_version>"3.9" and platform_system != "Darwin"',
    ],
    exclude_patterns = [
    'build/*'
    ],
    entry_points={
        'console_scripts': [
            'help=aimotion_f1tenth_utils.docs:open_docs'
        ]
    },
    html_static_path = ['images'],
)
