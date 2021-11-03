from distutils.core import setup
from Cython.Build import cythonize
import numpy

# os.environ['CFLAGS'] = '-O3'

setup(
    name='wpg',
    ext_modules=cythonize([
        "wpg/combinedqp.py", "wpg/base.py", "wpg/helper.py", "wpg/interpolation.py", "wpg/__init__.py",
        "nmpc_vel_ref_class.pyx"
    ],
                          language='c++',
                          compiler_directives={'language_level': '3'}),
    include_dirs=[numpy.get_include()],
)
