from setuptools import setup, find_packages

setup(
    name='pyolp_robotics',
    version='0.0.1',    
    description='A full python framework for robot offline progamming',
    url='https://github.com/Tanneguydv/pyolp_robotics',
    author='Tanneguy de Villemagne',
    author_email='tanneguydv@gmail.com',
    license='LGPL-3',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    include_package_data=True,
    package_data={'pyolp_robotics': ['robots/resources/**/**/*.stp']},
    classifiers=[
        'Development Status :: 1 - Planning',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
)
