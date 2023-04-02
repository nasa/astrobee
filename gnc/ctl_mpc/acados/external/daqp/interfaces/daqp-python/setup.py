from setuptools import setup,find_packages

setup(name='daqp',
        version='0.0',
        description='DAQP: A dual active-set QP solver',
        url='http://github.com/darnstrom/daqp',
        author='Daniel Arnström',
        author_email='daniel.arnstrom@liu.se',
        license='',
        packages=find_packages(
            where='src',
            include=['daqp']),
        package_dir={"": "src"},
        zip_safe=False)
