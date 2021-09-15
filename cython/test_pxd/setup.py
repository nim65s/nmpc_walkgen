from distutils.core import setup
from Cython.Build import cythonize

setup(name='libs',
      ext_modules=cythonize(["libs/a.py", "libs/__init__.py", "b.pyx"],
                            language='c++',
                            compiler_directives={'language_level': '3'}))
