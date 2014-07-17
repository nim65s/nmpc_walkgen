import os
import numpy
from numpy.testing import *

from walking_generator.base import BaseGenerator as Generator

BASEDIR = os.path.dirname(os.path.abspath(__file__))

class TestBaseGenerator(TestCase):
    """
    Test if BaseGenerator is assembling everything correctly
    """
    def test_fixed_model_matrices(self):
        gen = Generator()
        # NOTE usage: assert_allclose(actual, desired, rtol, atol, err_msg, verbose)

        T = gen.T
        h_com = gen.h_com
        g = gen.g

        for i in range(gen.N):
            assert_allclose(gen.Pzs[i,:], (1, i*T, i**2*T**2/2 + h_com/g))
            assert_allclose(gen.Pps[i,:], (1, i*T, i**2*T**2/2))
            assert_allclose(gen.Pvs[i,:], (0,       1, i*T))
            assert_allclose(gen.Pas[i,:], (0,       0,       1))

            for j in range(gen.N):
                if j <= i:
                    pass
                    assert_allclose(gen.Pzu[i,j], (3*(i-j)**2 + 3*(i-j) + 1)*T**3/6. + T*h_com/g)
                    assert_allclose(gen.Ppu[i,j], (3*(i-j)**2 + 3*(i-j) + 1)*T**3/6.)
                    assert_allclose(gen.Pvu[i,j], (2*(i-j) + 1)*T**2/2)
                    assert_allclose(gen.Pau[i,j], T)
                else:
                    assert_allclose(gen.Pzu[i,j], 0.0)
                    assert_allclose(gen.Ppu[i,j], 0.0)
                    assert_allclose(gen.Pvu[i,j], 0.0)
                    assert_allclose(gen.Pau[i,j], 0.0)

    def test_all_zero_when_idle(self):
        gen = Generator()
        # NOTE usage: assert_allclose(actual, desired, rtol, atol, err_msg, verbose)

        assert_allclose(gen.c_k_x, 0.0)
        assert_allclose(gen.c_k_y, 0.0)
        assert_allclose(gen.c_k_q, 0.0)
        assert_allclose(gen.C_kp1_x, 0.0)
        assert_allclose(gen.C_kp1_y, 0.0)
        assert_allclose(gen.C_kp1_q, 0.0)
        assert_allclose(gen.dC_kp1_x, 0.0)
        assert_allclose(gen.dC_kp1_y, 0.0)
        assert_allclose(gen.dC_kp1_q, 0.0)
        assert_allclose(gen.ddC_kp1_x, 0.0)
        assert_allclose(gen.ddC_kp1_y, 0.0)
        assert_allclose(gen.ddC_kp1_q, 0.0)
        assert_allclose(gen.dddC_k_x, 0.0)
        assert_allclose(gen.dddC_k_y, 0.0)
        assert_allclose(gen.dddC_k_q, 0.0)

        gen.simulate()

        assert_allclose(gen.c_k_x, 0.0)
        assert_allclose(gen.c_k_y, 0.0)
        assert_allclose(gen.c_k_q, 0.0)
        assert_allclose(gen.C_kp1_x, 0.0)
        assert_allclose(gen.C_kp1_y, 0.0)
        assert_allclose(gen.C_kp1_q, 0.0)
        assert_allclose(gen.dC_kp1_x, 0.0)
        assert_allclose(gen.dC_kp1_y, 0.0)
        assert_allclose(gen.dC_kp1_q, 0.0)
        assert_allclose(gen.ddC_kp1_x, 0.0)
        assert_allclose(gen.ddC_kp1_y, 0.0)
        assert_allclose(gen.ddC_kp1_q, 0.0)
        assert_allclose(gen.dddC_k_x, 0.0)
        assert_allclose(gen.dddC_k_y, 0.0)
        assert_allclose(gen.dddC_k_q, 0.0)

if __name__ == '__main__':
    try:
        import nose
        nose.runmodule()
    except ImportError:
        err_str = 'nose needed for unittests.\nPlease install using:\n   sudo pip install nose'
        raise ImportError(err_str)
