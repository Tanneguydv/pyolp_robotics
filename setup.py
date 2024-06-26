from setuptools import setup, find_packages

setup(
    name='pyolp_robotics',
    version='0.0.3',    
    description='A full python framework for robot offline progamming',
    url='https://github.com/Tanneguydv/pyolp_robotics',
    author='Tanneguy de Villemagne',
    author_email='tanneguydv@gmail.com',
    license='LGPL-3',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    include_package_data=True,
    package_data={'pyolp_robotics': ['robots/resources/**/**/*.stp', 'robots/resources/**/**/*.txt']},
    classifiers=[
        'Development Status :: 1 - Planning',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
    ],
)