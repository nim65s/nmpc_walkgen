import os
import numpy
from numpy.testing import *

from walking_generator import Generator

BASEDIR = os.path.dirname(os.path.abspath(__file__))

class TestPatternGenerator(TestCase):
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

        #gen.dddC_k_x[...] = 0.1
        #print gen.dddC_k_x

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

    def test_against_real_pattern_genererator_emergency_stop(self):
        # get test data
        print 'BASEDIR', BASEDIR
        data = numpy.loadtxt(os.path.join(BASEDIR, "data",
            "TestHerdt2010EmergencyStopTestFGPI.dat")
        )

        # 1 : NbOfIt*0.005
        # 2 : finalCOMPosition.x[0]
        # 3 : finalCOMPosition.y[0]
        # 4 : finalCOMPosition.z[0]
        # 5 : finalCOMPosition.yaw
        # 6 : finalCOMPosition.x[1]
        # 7 : finalCOMPosition.y[1]
        # 8 : finalCOMPosition.z[1]
        # 9 : ZMPTarget(0)
        #10 : ZMPTarget(1)
        #11 : LeftFootPosition.x
        #12 : LeftFootPosition.y
        #13 : LeftFootPosition.z
        #14 : LeftFootPosition.dx
        #15 : LeftFootPosition.dy
        #16 : LeftFootPosition.dz
        #17 : LeftFootPosition.ddx
        #18 : LeftFootPosition.ddy
        #19 : LeftFootPosition.ddz
        #20 : LeftFootPosition.theta
        #21 : LeftFootPosition.omega
        #22 : LeftFootPosition.omega2
        #23 : RightFootPosition.x
        #24 : RightFootPosition.y
        #25 : RightFootPosition.z
        #26 : RightFootPosition.dx
        #27 : RightFootPosition.dy
        #28 : RightFootPosition.dz
        #29 : RightFootPosition.ddx
        #30 : RightFootPosition.ddy
        #31 : RightFootPosition.ddz
        #32 : RightFootPosition.theta
        #33 : RightFootPosition.omega
        #34 : RightFootPosition.omega2

        gen = Generator()

    def test_against_real_pattern_genererator_online_walking(self):
        # get test data
        data = numpy.loadtxt(os.path.join(BASEDIR, "data",
            "TestHerdt2010OnLineTestFGPI.dat")
        )
        print data

        gen = Generator()

if __name__ == '__main__':
    try:
        import nose
        nose.runmodule()
    except ImportError:
        err_str = 'nose needed for unittests.\nPlease install using:\n   sudo pip install nose'
        raise ImportError(err_str)
