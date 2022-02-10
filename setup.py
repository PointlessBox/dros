from setuptools import setup

setup(
    name='dros',
    version='0.1.0',
    py_modules=['main', 'dros_utils', 'workspace', 'docker_commands', 'consts'],
    include_package_data=True,
    install_requires=[
        'Click',
        'Docker'
    ],
    entry_points={
        'console_scripts': [
            'dros = main:cli',
        ]
    }
)
