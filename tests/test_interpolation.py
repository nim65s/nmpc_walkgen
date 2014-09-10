import os
import numpy
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from numpy.testing import *
import scipy.linalg as linalg

from walking_generator.interpolation import *

BASEDIR = os.path.dirname(os.path.abspath(__file__))

class TestInterpolation(TestCase):
    """
    Test Interpolation Routines
    """
    #define tolerance for unittests
    ATOL = 1e-07
    RTOL = 1e-07

    def test_dummy(self):
        pass

if __name__ == '__main__':
    try:
        import nose
        nose.runmodule()
    except ImportError:
        err_str = 'nose needed for unittests.\nPlease install using:\n   sudo pip install nose'
        raise ImportError(err_str)
