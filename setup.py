from setuptools import setup

setup(
    name='dros',
    version='0.1.0',
    py_modules=['dros'],
    install_requires=[
        'Click',
        'Docker'
    ],
    entry_points={
        'console_scripts': [
            'dros = dros:cli',
        ]
    }
)
