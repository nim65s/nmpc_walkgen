import os
import numpy
from numpy.testing import *

from walking_generator.base import BaseGenerator as Generator
from walking_generator.base import BaseTypeFoot

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

    def test_basetypefoot_initialization(self):
        gen = Generator()

        currentSupport = BaseTypeFoot()
        supportDeque = numpy.empty( (gen.N,) , dtype=object )
        for i in range(gen.N):
            supportDeque[i] = BaseTypeFoot()

        U_kp1 =  numpy.hstack((gen.v_kp1.reshape(gen.v_kp1.size,1), gen.V_kp1))

        if (gen.currentSupport.foot == "left" ) :
            pair = "left"
            impair = "right"
        else :
            pair = "right"
            impair = "left"

        for i in range(gen.N):
            for j in range(U_kp1.shape[1]):
                if U_kp1[i][j] == 1 :
                    supportDeque[i].stepNumber = j
                    if (j % 2) == 0:
                        supportDeque[i].foot = pair
                    else :
                        supportDeque[i].foot = impair
            if i > 0 :
                supportDeque[i].ds = supportDeque[i].stepNumber -\
                                     supportDeque[i-1].stepNumber

        assert_equal(gen.currentSupport, currentSupport)
        assert_array_equal(gen.supportDeque, supportDeque)

    def test_BaseTypeFoot_update(self):
        gen = Generator()

        currentSupport = BaseTypeFoot(gen.f_k_x, gen.f_k_y, gen.f_k_q, "left")
        supportDeque = numpy.empty( (gen.N,) , dtype=object )
        for i in range(gen.N):
            supportDeque[i] = BaseTypeFoot()

        for i in range(100):
            U_kp1 =  numpy.hstack((gen.v_kp1.reshape(gen.v_kp1.size,1), gen.V_kp1))

            if (gen.currentSupport.foot == "left" ) :
                pair = "left"
                impair = "right"
            else :
                pair = "right"
                impair = "left"

            for i in range(gen.N):
                for j in range(U_kp1.shape[1]):
                    if U_kp1[i][j] == 1 :
                        supportDeque[i].stepNumber = j
                        if (j % 2) == 0:
                            supportDeque[i].foot = pair
                        else :
                            supportDeque[i].foot = impair
                if i > 0 :
                    supportDeque[i].ds = supportDeque[i].stepNumber -\
                                         supportDeque[i-1].stepNumber

            assert_equal(gen.currentSupport, currentSupport)
            assert_array_equal(gen.supportDeque, supportDeque)

            assert_equal(gen.currentSupport, currentSupport)
            assert_array_equal(gen.supportDeque, supportDeque)

    def test_selection_matrix_initialization(self):
        gen = Generator()

        nf = gen.nf
        N = gen.N
        nstep = int(gen.T_step/gen.T) # time span of single support phase

        v_kp1 = numpy.zeros(gen.v_kp1.shape)
        V_kp1 = numpy.zeros(gen.V_kp1.shape)

        v_kp1 = numpy.delete(v_kp1,0,None)
        v_kp1 = numpy.append(v_kp1,[0],axis=0)

        if v_kp1[0]==0:
            v_kp1 = numpy.zeros((N,), dtype=int)
            V_kp1 = numpy.zeros((N,nf), dtype=int)
            nf = 1
            for i in range(nstep):
                v_kp1[i] = 1
            for i in range(N-nstep):
                v_kp1[i+nstep] = 0
            step = 0
            for i in range (N) :
                step = (int)( i / nstep )
                for j in range (nf):
                    V_kp1[i,j] = (int)( i+1>nstep and j==step-1)

        else:
            V_kp1 = numpy.delete(V_kp1,0,axis=0)
            tmp = numpy.zeros( (1,V_kp1.shape[1]) , dtype=float)
            V_kp1 = numpy.append(V_kp1, tmp, axis=0)
            for j in range(V_kp1.shape[1]) :
                if V_kp1[:,j].sum() < nstep :
                    V_kp1[V_kp1.shape[0]-1][j] = 1
                    break
                else :
                    V_kp1[V_kp1.shape[0]-1][j] = 0

        assert_array_equal(gen.v_kp1, v_kp1)
        assert_array_equal(gen.V_kp1, V_kp1)

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

    def test_constraint_matrices(self):
        gen = Generator()

        repos = numpy.DataSource()

        data_DX = numpy.genfromtxt("./tests/data/DX.dat",skip_header=0)
        data_DY = numpy.genfromtxt("./tests/data/DY.dat",skip_header=0)

        ok = 1
        for i in range (data_DX.shape[0]):
            for j in range (data_DX.shape[1]):
                if data_DX[i,j] - gen.D_kp1x[i,j] == 0 :
                    print "no pb"
                else :
                    print "problem at " , i , "," , j
                    ok = 0
        if ok == 0 :
            print "##################### warning ###################################"
        print "ok = " , ok
        assert_allclose(data_DX, gen.D_kp1x, atol=1.e-4, rtol=1.e-4)
        assert_allclose(data_DY, gen.D_kp1y, atol=1.e-4, rtol=1.e-4)

        A = numpy.genfromtxt("./tests/data/A.dat",skip_header=1)
        lbA = numpy.genfromtxt("./tests/data/lbA.dat",skip_header=1)

        data_A_jx = A[ 1:(A.shape[0]-1) , 0:gen.N ]
        gen_A_jx = -gen.Acop[ : , 0:gen.N ]
        gen_A_jx = numpy.concatenate( (gen_A_jx , -gen.Afoot[0:gen.A0l.shape[0] , 0:gen.N]) )

        assert_allclose(data_A_jx, gen_A_jx)

        data_A_jy
        gen_A_jy

        data_A_fx
        gen_A_fx

        data_A_fy
        gen_A_fy






        genA = numpy.zeros( (1,2*gen.N+gen.nf) , dtype=float )
        genA = numpy.concatenate( (genA,-gen.Acop[: , 0:(2*gen.N+gen.nf)]) )
        genA = numpy.concatenate( (genA,-gen.Afoot[0:gen.A0l.shape[0] , 0:(2*gen.N+gen.nf)]) )
        genA = numpy.concatenate( (genA,numpy.zeros( (1,2*gen.N+gen.nf) , dtype=float )) )

        dimlbA = 129
        genlbA = [0]
        genlbA = numpy.concatenate( (genlbA,gen.ubBcop) )
        genlbA = numpy.concatenate( (genlbA,gen.Bfoot[0:gen.A0l.shape[0]]) )
        genlbA = numpy.concatenate( (genlbA,numpy.zeros( (129-genlbA.shape[0]-1,) , dtype=float )) )

        assert_allclose(genlbA, lbA)
        assert_allclose(genA, A)



if __name__ == '__main__':
    try:
        import nose
        nose.runmodule()
    except ImportError:
        err_str = 'nose needed for unittests.\nPlease install using:\n   sudo pip install nose'
        raise ImportError(err_str)
