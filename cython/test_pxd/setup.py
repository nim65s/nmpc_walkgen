from distutils.core import setup
from Cython.Build import cythonize

setup(
    name='test',
    ext_modules=cythonize("b.pyx",language='c++',compiler_directives={'language_level': '3'}))