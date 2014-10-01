import os
import numpy
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from numpy.testing import *
import scipy.linalg as linalg
import matplotlib.pyplot as plt

from walking_generator.visualization import Plotter
from walking_generator.classic import ClassicGenerator
from walking_generator.combinedqp import NMPCGenerator
from walking_generator.utility import color_matrix
try:
    from qpoases import PyOptions as Options
    from qpoases import PyPrintLevel as PrintLevel
    from qpoases import PySQProblem as SQProblem
    from qpoases import PySolutionAnalysis as SolutionAnalysis
except ImportError:
    err_str = 'Please install qpOASES python interface, else you will not be able to use this pattern generator.'
    raise ImportError(err_str)

BASEDIR = os.path.dirname(os.path.abspath(__file__))

#define global tolerance for unittests
ATOL = 1e-06
RTOL = 1e-06

class TestNMPCGenerator(TestCase):
    """
    Test classic pattern generator, also against results from LAAS
    """

    def test_assembly_of_dofs(self):
        # define initial values
        comx = [0.00949035, 0.0, 0.0]
        comy = [0.095,      0.0, 0.0]
        comz = 0.814
        footx = 0.00949035
        footy = 0.095
        footq = 0.0

        # pattern generator preparation
        gen = NMPCGenerator()

        # set reference velocities to zero
        gen.set_velocity_reference([0.2,0.0,-0.2])
        gen.set_security_margin(0.04, 0.04)

        # set initial values
        gen.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

        # hack generator
        gen.nc = 0

        # setup problem
        gen.dofs = numpy.zeros(gen.nv)
        gen.qp   = SQProblem(gen.nv, gen.nc)

        # load NMPC options
        #gen.qp.setOptions(gen.options)

        gen.H   =  numpy.eye(gen.nv,gen.nv)
        gen.A   =  numpy.zeros((gen.nc,gen.nv))
        gen.g   =  numpy.zeros((gen.nv,))
        gen.lb  = -numpy.ones((gen.nv,))*1e+08
        gen.ub  =  numpy.ones((gen.nv,))*1e+08
        gen.lbA = -numpy.ones((gen.nc,))*1e+08
        gen.ubA =  numpy.ones((gen.nc,))*1e+08

        gen._qp_is_initialized = False

        # renaming for convenience
        N  = gen.N
        nf = gen.nf

        # define specified input
        gen.dddC_k_x [...] = 1.0
        gen.F_k_x    [...] = 2.0
        gen.dddC_k_y [...] = 3.0
        gen.F_k_y    [...] = 4.0
        gen.dddF_k_qR[...] = 5.0
        gen.dddF_k_qL[...] = 6.0

        dddC_k_x  = gen.dddC_k_x .copy()
        F_k_x     = gen.F_k_x    .copy()
        dddC_k_y  = gen.dddC_k_y .copy()
        F_k_y     = gen.F_k_y    .copy()
        dddF_k_qR = gen.dddF_k_qR.copy()
        dddF_k_qL = gen.dddF_k_qL.copy()

        # do an assembly of the problem
        gen._preprocess_solution()

        # check if dofs are assembled correctly
        assert_allclose(gen.dofs[0  :0+N   ], gen.dddC_k_x)
        assert_allclose(gen.dofs[0+N:0+N+nf], gen.F_k_x)

        a = N+nf
        assert_allclose(gen.dofs[a  :a+N   ], gen.dddC_k_y)
        assert_allclose(gen.dofs[a+N:a+N+nf], gen.F_k_y)

        a = 2*(N+nf)
        assert_allclose(gen.dofs[  a:a+N], gen.dddF_k_qR)
        assert_allclose(gen.dofs[ -N:   ], gen.dddF_k_qL)

        # reset initilization
        gen.H  [...] =  numpy.eye(gen.nv,gen.nv)
        gen.A  [...] =  numpy.zeros((gen.nc,gen.nv))
        gen.g  [...] =  numpy.zeros((gen.nv,))
        gen.lb [...] = -numpy.ones((gen.nv,))*1e+08
        gen.ub [...] =  numpy.ones((gen.nv,))*1e+08
        gen.lbA[...] = -numpy.ones((gen.nc,))*1e+08
        gen.ubA[...] =  numpy.ones((gen.nc,))*1e+08

        # calculate solution which shall be equal to zero
        #gen._solve_qp()
        #assert_allclose(gen.dofs, 0.0, atol=ATOL, rtol=RTOL)

        # hack solution
        gen.dofs[0  :0+N   ] = 1.0
        gen.dofs[0+N:0+N+nf] = 2.0

        a = N+nf
        gen.dofs[a  :a+N   ] = 3.0
        gen.dofs[a+N:a+N+nf] = 4.0

        a = 2*(N+nf)
        gen.dofs[  a:a+N] = 5.0
        gen.dofs[ -N:   ] = 6.0

        # adjust reference values
        dddC_k_x [...] =  2.0
        F_k_x    [...] =  4.0
        dddC_k_y [...] =  6.0
        F_k_y    [...] =  8.0
        dddF_k_qR[...] = 10.0
        dddF_k_qL[...] = 12.0

        # add increment to
        gen._postprocess_solution()

        # check if dofs are assembled correctly
        assert_allclose(gen.dddC_k_x, dddC_k_x)
        assert_allclose(gen.F_k_x, F_k_x)

        a = N+nf
        assert_allclose(gen.dddC_k_y, dddC_k_y)
        assert_allclose(gen.F_k_y, F_k_y)

        a = 2*(N+nf)
        assert_allclose(gen.dddF_k_qR, dddF_k_qR)
        assert_allclose(gen.dddF_k_qL, dddF_k_qL)

    def test_compare_submatrices_to_classic_generator(self):
        # define initial values
        comx = [0.00949035, 0.0, 0.0]
        comy = [0.095,      0.0, 0.0]
        comz = 0.814
        footx = 0.00949035
        footy = 0.095
        footq = 0.0

        # Pattern Generator Preparation
        classic = ClassicGenerator()

        # set reference velocities to zero
        classic.set_velocity_reference([0.2,0.0,-0.2])
        classic.set_security_margin(0.04, 0.04)

        # set initial values
        classic.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

        # build up QP matrices
        classic._preprocess_solution()

        # reference them for comparison
        pos_H = classic.pos_H
        classic_Q_k_x = pos_H[:classic.N+classic.nf, :classic.N+classic.nf]
        classic_Q_k_y = pos_H[-(classic.N+classic.nf):, -(classic.N+classic.nf):]

        pos_g = classic.pos_g
        classic_p_k_x = pos_g[:classic.N+classic.nf]
        classic_p_k_y = pos_g[-(classic.N+classic.nf):]

        ori_H = classic.ori_H
        classic_Q_k_qR = ori_H[:classic.N, :classic.N]
        classic_Q_k_qL = ori_H[-classic.N:, -classic.N:]

        ori_g = classic.ori_g
        classic_p_k_qR = ori_g[:classic.N]
        classic_p_k_qL = ori_g[-classic.N:]

        # define initial values
        comx = [0.00949035, 0.0, 0.0]
        comy = [0.095,      0.0, 0.0]
        comz = 0.814
        footx = 0.00949035
        footy = 0.095
        footq = 0.0

        # Pattern Generator Preparation
        nmpc = NMPCGenerator()

        nmpc.set_velocity_reference([0.2,0.0,-0.2])
        nmpc.set_security_margin(0.04, 0.04)

        # set initial values
        nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

        # build up QP matrices
        nmpc._preprocess_solution()

        # reference them for comparison
        nmpc_Q_k_x = nmpc.Q_k_x
        nmpc_p_k_x = nmpc.p_k_x
        nmpc_p_k_y = nmpc.p_k_y

        nmpc_Q_k_qR = nmpc.Q_k_qR
        nmpc_Q_k_qL = nmpc.Q_k_qL
        nmpc_p_k_qR = nmpc.p_k_qR
        nmpc_p_k_qL = nmpc.p_k_qL

        # compare matrices
        # position common sub expressions
        assert_allclose(classic_Q_k_x, nmpc_Q_k_x, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_Q_k_y, nmpc_Q_k_x, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_p_k_x, nmpc_p_k_x, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_p_k_y, nmpc_p_k_y, atol=ATOL, rtol=RTOL)

        # orientation common sub expressions
        assert_allclose(classic_Q_k_qR, nmpc_Q_k_qR, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_Q_k_qL, nmpc_Q_k_qL, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_p_k_qR, nmpc_p_k_qR, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_p_k_qL, nmpc_p_k_qL, atol=ATOL, rtol=RTOL)

    def test_compare_constraint_matrices_to_classic_generator(self):
        # define initial values
        comx = [0.00949035, 0.0, 0.0]
        comy = [0.095,      0.0, 0.0]
        comz = 0.814
        footx = 0.00949035
        footy = 0.095
        footq = 0.0

        # Pattern Generator Preparation
        classic = ClassicGenerator(fsm_state='R/L')

        # set reference velocities to zero
        classic.set_velocity_reference([0.2,0.0,-0.2])
        classic.set_security_margin(0.04, 0.04)

        # set initial values
        classic.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

        # build up QP matrices
        classic._preprocess_solution()

        # reference them for comparison
        classic_pos_A   = classic.pos_A
        classic_pos_lbA = classic.pos_lbA
        classic_pos_ubA = classic.pos_ubA

        classic_ori_A   = classic.ori_A
        classic_ori_lbA = classic.ori_lbA
        classic_ori_ubA = classic.ori_ubA

        # define initial values
        comx = [0.00949035, 0.0, 0.0]
        comy = [0.095,      0.0, 0.0]
        comz = 0.814
        footx = 0.00949035
        footy = 0.095
        footq = 0.0

        # Pattern Generator Preparation
        nmpc = NMPCGenerator(fsm_state='R/L')

        nmpc.set_velocity_reference([0.2,0.0,-0.2])
        nmpc.set_security_margin(0.04, 0.04)

        # set initial values
        nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

        # build up QP matrices
        nmpc._preprocess_solution()

        # reference them for comparison
        nmpc_pos_A   = nmpc.A_pos_x
        nmpc_pos_lbA = nmpc.lbA_pos
        nmpc_pos_ubA = nmpc.ubA_pos

        nmpc_ori_A   = nmpc.A_ori
        nmpc_ori_lbA = nmpc.lbA_ori
        nmpc_ori_ubA = nmpc.ubA_ori

        nmpc_A_pos   = nmpc.qp_A  [:nmpc.nc_pos,:2*(nmpc.N+nmpc.nf)]
        nmpc_lbA_pos = nmpc.qp_lbA[:nmpc.nc_pos]
        nmpc_ubA_pos = nmpc.qp_ubA[:nmpc.nc_pos]

        nmpc_A_ori   = nmpc.qp_A  [-nmpc.nc_ori:,-2*nmpc.N:]
        nmpc_lbA_ori = nmpc.qp_lbA[-nmpc.nc_ori:]
        nmpc_ubA_ori = nmpc.qp_ubA[-nmpc.nc_ori:]

        # compare matrices
        # position common sub expressions
        assert_allclose(classic_pos_A,   nmpc_pos_A,   atol=ATOL, rtol=RTOL)
        assert_allclose(classic_pos_lbA, nmpc_pos_lbA, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_pos_ubA, nmpc_pos_ubA, atol=ATOL, rtol=RTOL)

        assert_allclose(classic_pos_A,   nmpc_A_pos,   atol=ATOL, rtol=RTOL)
        assert_allclose(classic_pos_lbA, nmpc_lbA_pos, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_pos_ubA, nmpc_ubA_pos, atol=ATOL, rtol=RTOL)

        # orientation common sub expressions
        assert_allclose(classic_ori_A,   nmpc_ori_A,   atol=ATOL, rtol=RTOL)
        assert_allclose(classic_ori_lbA, nmpc_ori_lbA, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_ori_ubA, nmpc_ori_ubA, atol=ATOL, rtol=RTOL)

        assert_allclose(classic_ori_A,   nmpc_A_ori,   atol=ATOL, rtol=RTOL)
        assert_allclose(classic_ori_lbA, nmpc_lbA_ori, atol=ATOL, rtol=RTOL)
        assert_allclose(classic_ori_ubA, nmpc_ubA_ori, atol=ATOL, rtol=RTOL)

    def test_new_generator(self):
        # define initial values
        comx = [0.00949035, 0.0, 0.0]
        comy = [0.095,      0.0, 0.0]
        comz = 0.814
        footx = 0.00949035
        footy = 0.095
        footq = 0.0

        # Pattern Generator Preparation
        nmpc = NMPCGenerator()

        nmpc.set_velocity_reference([0.2,0.0, 0.0])
        nmpc.set_security_margin(0.04, 0.04)

        # set initial values
        nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

        # build up QP matrices
        nmpc.solve()
        nmpc.update()
        print nmpc.qp_nwsr
        print nmpc.qp_cputime

        nmpc.solve()
        nmpc.update()

        print nmpc.qp_nwsr
        print nmpc.qp_cputime


if __name__ == '__main__':
    try:
        import nose
        nose.runmodule()
    except ImportError:
        err_str = 'nose needed for unittests.\nPlease install using:\n   sudo pip install nose'
        raise ImportError(err_str)
