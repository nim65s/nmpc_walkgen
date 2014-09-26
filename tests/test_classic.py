import os
import numpy
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from numpy.testing import *
import scipy.linalg as linalg
import matplotlib.pyplot as plt

from walking_generator.classic import ClassicGenerator
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

# Interpolation Data Mapping
names = {
     0 : '     Time',
     1 : '     CoMx',
     2 : '     CoMy',
     3 : '     CoMz',
     4 : '     CoMq',
     5 : '    dCoMx',
     6 : '    dCoMy',
     7 : '    dCoMz',
     8 : '    dCoMq',
     9 : '   ddCoMx',
    10 : '   ddCoMy',
    11 : '   ddCoMz',
    12 : '   ddCoMq',
    13 : ' CoPx_ref',
    14 : ' CoPy_ref',
    15 : '  LFoot_x',
    16 : '  LFoot_y',
    17 : '  LFoot_z',
    18 : ' dLFoot_x',
    19 : ' dLFoot_y',
    20 : ' dLFoot_z',
    21 : 'ddLFoot_x',
    22 : 'ddLFoot_y',
    23 : 'ddLFoot_z',
    24 : '  LFoot_q',
    25 : ' dLFoot_q',
    26 : 'ddLFoot_q',
    27 : '  RFoot_x',
    28 : '  RFoot_y',
    29 : '  RFoot_z',
    30 : ' dRFoot_x',
    31 : ' dRFoot_y',
    32 : ' dRFoot_z',
    33 : 'ddRFoot_x',
    34 : 'ddRFoot_y',
    35 : 'ddRFoot_z',
    36 : '  RFoot_q',
    37 : ' dRFoot_q',
    38 : 'ddRFoot_q',
}

#define global tolerance for unittests
ATOL = 1e-06
RTOL = 1e-06

class TestClassicGenerator(TestCase):
    """
    Test classic pattern generator, also against results from LAAS
    """

    def test_qp_setup_with_toy_example(self):
        gen = ClassicGenerator()

        # define input matrices
        gen.pos_H[...] = numpy.eye(gen.pos_nv)
        gen.pos_g[...] = numpy.ones((gen.pos_nv,))

        gen.pos_lb[...] = -numpy.ones((gen.pos_nv,))*0.5
        gen.pos_ub[...] =  numpy.ones((gen.pos_nv,))*0.5

        gen.pos_A[...]   = numpy.eye(gen.pos_nc, gen.pos_nv) + numpy.eye(gen.pos_nc, gen.pos_nv, k=1)
        gen.pos_lbA[...] = -numpy.ones((gen.pos_nc,))
        gen.pos_ubA[...] =  numpy.ones((gen.pos_nc,))

        # define input matrices
        gen.ori_H[...] = numpy.eye(gen.ori_nv)
        gen.ori_g[...] = numpy.ones((gen.ori_nv,))

        gen.ori_lb[...] = -numpy.ones((gen.ori_nv,))*0.5
        gen.ori_ub[...] =  numpy.ones((gen.ori_nv,))*0.5

        gen.ori_A[...]   = numpy.eye(gen.ori_nc, gen.ori_nv) + numpy.eye(gen.ori_nc, gen.ori_nv, k=1)
        gen.ori_lbA[...] = -numpy.ones((gen.ori_nc,))
        gen.ori_ubA[...] =  numpy.ones((gen.ori_nc,))

        # define solution
        ori_x = -numpy.ones((gen.ori_nv,))*0.5
        ori_f = -12.0
        pos_x = -numpy.ones((gen.pos_nv,))*0.5
        pos_f = -13.5

        # test first qp solution
        gen._solve_qp()
        gen._postprocess_solution()

        # get solution
        assert_allclose(gen.ori_dofs, ori_x, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.ori_qp.getObjVal(), ori_f, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.pos_dofs, pos_x, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.pos_qp.getObjVal(), pos_f, rtol=RTOL, atol=ATOL)

    def test_qp_setup_with_toy_example_hack_from_qpoases_manual(self):
        gen = ClassicGenerator()

        options = Options()
        options.printLevel = PrintLevel.LOW


        # define matrices from qpoases manual
        H   = numpy.array([ 1.0, 0.0, 0.0, 0.5 ]).reshape((2,2))
        A   = numpy.array([ 1.0, 1.0 ]).reshape((1,2))
        g   = numpy.array([ 1.5, 1.0 ])
        lb  = numpy.array([ 0.5, -2.0 ])
        ub  = numpy.array([ 5.0, 2.0 ])
        lbA = numpy.array([ -1.0 ])
        ubA = numpy.array([ 2.0 ])

        x = numpy.array([0.5, -1.5])
        f = -6.25e-02

        # hack pattern generator
        gen.ori_nv = 2
        gen.ori_nc = 1
        gen.ori_dofs = numpy.zeros((2,))
        gen.ori_qp = SQProblem(gen.ori_nv, gen.ori_nc)
        gen.ori_qp.setOptions(options)

        gen.ori_H   = H
        gen.ori_A   = A
        gen.ori_g   = g
        gen.ori_lb  = lb
        gen.ori_ub  = ub
        gen.ori_lbA = lbA
        gen.ori_ubA = ubA

        gen.pos_nv = 2
        gen.pos_nc = 1
        gen.pos_dofs = numpy.zeros((2,))
        gen.pos_qp = SQProblem(gen.pos_nv, gen.pos_nc)
        gen.pos_qp.setOptions(options)

        gen.pos_H   = H
        gen.pos_A   = A
        gen.pos_g   = g
        gen.pos_lb  = lb
        gen.pos_ub  = ub
        gen.pos_lbA = lbA
        gen.pos_ubA = ubA

        # test first qp solution
        gen._solve_qp()

        # get solution
        # NOTE post_process put entries into array that dont match anymore
        gen.pos_qp.getPrimalSolution(gen.pos_dofs)
        gen.ori_qp.getPrimalSolution(gen.ori_dofs)

        assert_allclose(gen.pos_dofs, x, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.pos_qp.getObjVal(), f, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.ori_dofs, x, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.ori_qp.getObjVal(), f, rtol=RTOL, atol=ATOL)

        # define matrices for warmstart
        H_new   = numpy.array([ 1.0, 0.5, 0.5, 0.5 ]).reshape((2,2))
        A_new   = numpy.array([ 1.0, 5.0 ]).reshape((1,2))
        g_new   = numpy.array([ 1.0, 1.5 ])
        lb_new  = numpy.array([ 0.0, -1.0 ])
        ub_new  = numpy.array([ 5.0, -0.5 ])
        lbA_new = numpy.array([ -2.0 ])
        ubA_new = numpy.array([ 1.0 ])

        x = numpy.array([0.5, -0.5])
        f = -1.875e-01

        # hack pattern generator
        gen.ori_H   = H_new
        gen.ori_A   = A_new
        gen.ori_g   = g_new
        gen.ori_lb  = lb_new
        gen.ori_ub  = ub_new
        gen.ori_lbA = lbA_new

        gen.pos_H   = H_new
        gen.pos_A   = A_new
        gen.pos_g   = g_new
        gen.pos_lb  = lb_new
        gen.pos_ub  = ub_new
        gen.pos_lbA = lbA_new

        # test qp warmstart
        gen._solve_qp()

        # get solution
        # NOTE post_process put entries into array that dont match anymore
        gen.pos_qp.getPrimalSolution(gen.pos_dofs)
        gen.ori_qp.getPrimalSolution(gen.ori_dofs)

        assert_allclose(gen.pos_dofs, x, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.pos_qp.getObjVal(), f, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.ori_dofs, x, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.ori_qp.getObjVal(), f, rtol=RTOL, atol=ATOL)

    def test_classic_generator_weights(self):
        # weights defined for the Heirdt algorithm
        a = 1.0   # weight for CoM velocity tracking
        b = 0.0   # weight for CoM average velocity tracking
        c = 1e-06 # weight for ZMP reference tracking
        d = 1e-05 # weight for jerk minimization

        gen = ClassicGenerator()

        assert_equal(gen.a, a)
        assert_equal(gen.b, b)
        assert_equal(gen.c, c)
        assert_equal(gen.d, d)

    def test_matrices_of_position_qp_objective(self):
        gen = ClassicGenerator()
        gen._preprocess_solution()

        # check symmetry of Hessian
        pos_H = gen.pos_H
        assert_allclose(pos_H - pos_H.transpose(), 0.0, rtol=RTOL, atol=ATOL)

        # check positive definiteness
        U, s, V = linalg.svd(pos_H)
        assert_equal((s > 0).all(), True)

        # test for equality of block ins pos_H
        pos_H_A = pos_H[ :gen.N+gen.nf, :gen.N+gen.nf]
        pos_H_B = pos_H[-gen.N-gen.nf:,-gen.N-gen.nf:]
        assert_allclose(pos_H_A, pos_H_B, rtol=RTOL, atol=ATOL)

    def test_qp_objective_gradient_against_real_pattern_generator(self):
        # instantiate pattern generator
        gen = ClassicGenerator(fsm_state='L/R')
        secmargin = 0.04
        gen.set_security_margin(secmargin, secmargin)

        comx = [0.06591456,0.07638739,-0.1467377]
        comy = [2.49008564e-02,6.61665254e-02,6.72712187e-01]
        comz = 0.814
        footx = 0.00949035
        footy = 0.095
        footq = 0.0
        gen.set_initial_values(comx, comy, comz, footx, footy, footq)

        # define reference velocity
        gen.dC_kp1_x_ref[...] = 0.2
        gen.dC_kp1_y_ref[...] = 0.0

        gen._preprocess_solution()

        N = gen.N
        nf = gen.nf

        # data follows other convention, i.e.
        # U_k = (dddC_x, dddC_y, F_x, F_y)

        pos_g = numpy.loadtxt(os.path.join(BASEDIR, "data", "P.dat"), skiprows=1)
        g_mask = numpy.zeros(gen.pos_g.shape, dtype=bool)

        # compare values for dddC_kp1_x
        g_mask[...] = 0
        g_mask[:N] = 1
        assert_allclose(
            gen.pos_g[g_mask],
            pos_g[:N],
            rtol=RTOL, atol=ATOL
        )

        # compare values for dddC_kp1_y
        g_mask[...] = 0
        g_mask[N+nf:-nf] = 1
        assert_allclose(
            gen.pos_g[g_mask],
            pos_g[N:2*N],
            rtol=RTOL, atol=ATOL
        )

        # compare values for F_k_x
        g_mask[...] = 0
        g_mask[N:N+nf-1] = 1
        assert_allclose(
            gen.pos_g[g_mask],
            pos_g[2*N:2*N+1],
            rtol=RTOL, atol=ATOL
        )

        # compare values for F_k_y
        g_mask[...] = 0
        g_mask[-nf:-1] = 1
        assert_allclose(
            gen.pos_g[g_mask],
            pos_g[2*N+1:],
            rtol=RTOL, atol=ATOL
        )

    def test_qp_objective_hessian_against_real_pattern_generator(self):
        # instantiate pattern generator
        gen = ClassicGenerator(fsm_state='L/R')
        gen._preprocess_solution()

        N = gen.N
        nf = gen.nf

        # data follows other convention, i.e.
        # U_k = (dddC_x, dddC_y, F_x, F_y)

        # assemble pos_H and pos_g for our convention
        pos_H = numpy.loadtxt(os.path.join(BASEDIR, "data", "Q.dat"), skiprows=1)
        H_mask = numpy.zeros(gen.pos_H.shape, dtype=bool)

        # compare values for dddC_kp1_x
        H_mask[...] = 0
        H_mask[:N, :N] = 1
        assert_allclose(
            gen.pos_H[H_mask].reshape((N,N)),
            pos_H    [:N, :N],
            rtol=RTOL, atol=ATOL
        )

        # compare values for dddC_kp1_y
        H_mask[...] = 0
        H_mask[N+nf:-nf, N+nf:-nf] = 1
        assert_allclose(
            gen.pos_H[H_mask].reshape((N,N)),
            pos_H    [N:2*N, N:2*N],
            rtol=RTOL, atol=ATOL
        )

        # compare values for F_k_x
        H_mask[...] = 0
        H_mask[N:N+nf-1, N:N+nf-1] = 1
        assert_allclose(
            gen.pos_H[H_mask].reshape((1,1)),
            pos_H    [2*N:2*N+1, 2*N:2*N+1],
            rtol=RTOL, atol=ATOL
        )

        # compare values for F_k_y
        H_mask[...] = 0
        H_mask[2*N+nf:-1, 2*N+nf:-1] = 1
        assert_allclose(
            gen.pos_H[H_mask].reshape((1,1)),
            pos_H    [2*N+1:, 2*N+1:],
            rtol=RTOL, atol=ATOL
        )

    def test_qp_constraint_setup_against_real_pattern_generator(self):
        # instantiate pattern generator
        gen = ClassicGenerator(fsm_state='L/R')
        secmargin = 0.04
        gen.set_security_margin(secmargin, secmargin)

        # define initial state
        comx = [0.06591456,0.07638739,-0.1467377]
        comy = [2.49008564e-02,6.61665254e-02,6.72712187e-01]
        comz = 0.814
        footx = 0.00949035
        footy = 0.095
        footq = 0.0
        gen.set_initial_values(comx, comy, comz, footx, footy, footq)

        # data follows other convention, i.e.
        # U_k = (dddC_x, dddC_y, F_x, F_y)

        # get box constraints from data
        pos_lb = numpy.loadtxt(os.path.join(BASEDIR, "data", "LB.dat"), skiprows=1)
        pos_ub = numpy.loadtxt(os.path.join(BASEDIR, "data", "UB.dat"), skiprows=1)

        # get linear constraints from data
        pos_lbA = numpy.loadtxt(os.path.join(BASEDIR, "data", "lbA.dat"), skiprows=1)[1:70]

        # setup QP matrices
        gen._preprocess_solution()

        # test box constraints
        assert_allclose(gen.pos_lb[:34], pos_lb, rtol=RTOL, atol=ATOL)
        assert_allclose(gen.pos_ub[:34], pos_ub, rtol=RTOL, atol=ATOL)

        # test linear constraints
        assert_allclose(gen.pos_ubA[:-gen.pos_nc_eqfoot-gen.nFootPosHullEdges], pos_lbA, rtol=RTOL, atol=ATOL)

        gen.simulate()

    def test_generator_with_zero_reference_velocity(self):
        gen = ClassicGenerator()

        # set reference velocities to zero
        gen.dC_kp1_x_ref[...] = 0.0
        gen.dC_kp1_y_ref[...] = 0.0
        gen.dC_kp1_q_ref[...] = 0.0

        for i in range(100):
            print 'iteration: i = ', i
            gen.solve()
            gen.update()
            gen.simulate()

            gen.c_k_x[0] = gen.  C_kp1_x[0]
            gen.c_k_x[1] = gen. dC_kp1_x[0]
            gen.c_k_x[2] = gen.ddC_kp1_x[0]

            gen.c_k_y[0] = gen.  C_kp1_y[0]
            gen.c_k_y[1] = gen. dC_kp1_y[0]
            gen.c_k_y[2] = gen.ddC_kp1_y[0]

            gen.c_k_q[0] = gen.  C_kp1_q[0]
            gen.c_k_q[1] = gen. dC_kp1_q[0]
            gen.c_k_q[2] = gen.ddC_kp1_q[0]

            RTOL = 1e-05
            ATOL = 1e-05

            #assert_allclose(gen.dddC_k_x, 0.0, rtol=RTOL, atol=ATOL)
            #assert_allclose(gen.dddC_k_y, 0.0, rtol=RTOL, atol=ATOL)
            #assert_allclose(gen.dddC_k_q, 0.0, rtol=RTOL, atol=ATOL)

            #assert_allclose(gen.F_k_x, 0.0, rtol=RTOL, atol=ATOL)
            #assert_allclose(gen.F_k_y, 0.0, rtol=RTOL, atol=ATOL)
            #assert_allclose(gen.F_k_q, 0.0, rtol=RTOL, atol=ATOL)

    def test_real_pattern_genererator_data(self):
        """ verify if data is reproducible """
        interp_data = numpy.loadtxt(
            os.path.join(BASEDIR, "data", "walkForward2m_sInterpolation.dat")
        )

        # instantiate pattern generator
        gen = ClassicGenerator()

        # index of 3 step after idle motion
        # 5.0s : idle time
        # 5.8s : first step
        # 6.6s : second step
        # 7.4s : third step ('left')
        index = 1479
        for i in range(1):
            qp_idx = index + i
            ini_idx = index + i
            qp_dofs = interp_data[qp_idx, -(2*gen.N+2*gen.nf):]

            # extract states
            c_k_x    = interp_data[ini_idx,  1]
            c_k_y    = interp_data[ini_idx,  2]
            c_k_q    = interp_data[ini_idx,  4]
            dc_k_x   = interp_data[ini_idx,  5]
            dc_k_y   = interp_data[ini_idx,  6]
            dc_k_z   = interp_data[ini_idx,  7]
            dc_k_q   = interp_data[ini_idx,  8]
            h_com    = interp_data[ini_idx,  3]
            ddc_k_x  = interp_data[ini_idx,  9]
            ddc_k_y  = interp_data[ini_idx, 10]
            ddc_k_z  = interp_data[ini_idx, 11]
            ddc_k_q  = interp_data[ini_idx, 12]

            comx = (c_k_x, dc_k_x, ddc_k_x)
            comy = (c_k_y, dc_k_y, ddc_k_y)
            comq = (c_k_q, dc_k_q, ddc_k_q)
            comz = h_com
            foot_x = 0.0
            foot_y = 0.0
            foot_q = 0.0

            stuff = gen.update()
            foot  = stuff[-2]
            com_q=(0,0,0)

            # define initial values
            #gen.set_initial_values(
            #)

            # check if current ZMP is calculated correctly
            z_k_x    = interp_data[ini_idx, 13]
            z_k_y    = interp_data[ini_idx, 14]
            assert_allclose(gen.z_k_x, z_k_x)
            assert_allclose(gen.z_k_y, z_k_y)

            # extract single dofs from data block
            dddC_k_x = qp_dofs[             :gen.N       ]
            F_k_x    = qp_dofs[        gen.N:gen.N+gen.nf]
            dddC_k_y = qp_dofs[-gen.N-gen.nf:     -gen.nf]
            F_k_y    = qp_dofs[      -gen.nf:            ]

        print 'dddC_k_x\n', dddC_k_x
        print 'dddC_k_y\n', dddC_k_y
        print 'F_k_x   \n', F_k_x
        print 'F_k_y   \n', F_k_y


        return 0

        # set reference velocities
        gen.dC_kp1_x_ref[...] = 0.2
        gen.dC_kp1_y_ref[...] = 0.0
        gen.dC_kp1_q_ref[...] = 0.0

        # initialize data
        gen.set_initial_values(
            comx,comy,comz,
            supportfootx,supportfooty,supportfootq
        )

    def test_against_real_pattern_genererator_walkForward2m_s_while_walking(self):
        """
        Take data where robot is walking with constant CoM velocity and start
        test from there. This surpasses FSM implementation and starting and
        stopping maneuvers.
        """
        interp_data = numpy.loadtxt(
            os.path.join(BASEDIR, "data", "walkForward2m_sInterpolation.dat")
        )

        if True:
            for i in range(interp_data.shape[0])[1:]:
                if interp_data[i,0] == 7.4:
                    print 'i = ', i

        # instantiate pattern generator in walking mode,
        # i.e. in single support mode
        gen = ClassicGenerator(fsm_state='R/L')

        # define constant x CoM velocity to track
        gen.dC_kp1_x_ref[...] = 0.2
        gen.dC_kp1_y_ref[...] = 0.0
        gen.dC_kp1_q_ref[...] = 0.0

        # take walking initial state from interpolation data
        # NOTE we take the second step as initial state for the test, because
        #      this is a nearly periodic solution

        # idx is index on states used from interpolation data
        idx = 1479
        comx = (interp_data[idx,1], interp_data[idx,5], interp_data[idx, 9])
        comy = (interp_data[idx,2], interp_data[idx,6], interp_data[idx,10])
        comz = interp_data[idx,3]
        supportfootx = interp_data[idx, 15]
        supportfooty = interp_data[idx, 16]
        supportfootq = interp_data[idx, 24]
        gen.set_initial_values(
            comx,comy,comz,
            supportfootx,supportfooty,supportfootq
        )

        return 0

        idx = 69
        for i in range(10):
            print 'iteration = ', i

            # solve QP for solution
            gen.solve()

            # get reference values
            dddC_k_x_ref = qp_data[idx+i,            :gen.N       ]
            F_k_x_ref    = qp_data[idx+i,gen.N       :gen.N+gen.nf]

            dddC_k_y_ref = qp_data[idx+i,gen.N+gen.nf:2*gen.N+gen.nf]
            F_k_y_ref    = qp_data[idx+i,     -gen.nf:              ]

            # check orientation values, should be constant zero, because we
            # assume that robot does not rotate
            #assert_allclose(gen.dddC_k_q, 0.0)
            #assert_allclose(gen.F_k_q, 0.0)

            # check position DoFs against data from C++ implementation
            assert_allclose(gen.dddC_k_x, dddC_k_x_ref)
            assert_allclose(gen.dddC_k_y, dddC_k_y_ref)
            assert_allclose(gen.F_k_x,    F_k_x_ref)
            assert_allclose(gen.F_k_y,    F_k_y_ref)

            # simulate to get new values
            gen.simulate()

            # TODO shifting do I use interpolation or just take the value from
            #      simulation
            gen.c_k_x[...] = (gen.C_kp1_x[0], gen.dC_kp1_x[0], gen.ddC_kp1_x[0])
            gen.c_k_y[...] = (gen.C_kp1_y[0], gen.dC_kp1_y[0], gen.ddC_kp1_y[0])
            gen.c_k_q[...] = (gen.C_kp1_q[0], gen.dC_kp1_q[0], gen.ddC_kp1_q[0])

            gen.update()

    def test_against_real_pattern_genererator_walkForward2m_s(self):
        # get test data
        data = numpy.loadtxt(os.path.join(BASEDIR, "data",
            "walkForward2m_s.dat")
        )

        # instantiate pattern generator
        gen = ClassicGenerator()
        secmargin = 0.04
        gen.set_security_margin(secmargin, secmargin)

        comx = [0.06591456,0.07638739,-0.1467377]
        comy = [2.49008564e-02,6.61665254e-02,6.72712187e-01]
        comz = 0.814
        footx = 0.00949035
        footy = 0.095
        footq = 0.0
        gen.set_initial_values(comx, comy, comz, footx, footy, footq)

        # define reference velocity
        dC_kp1_x_ref = numpy.zeros((data.shape[0], gen.N), dtype=float)
        dC_kp1_y_ref = numpy.zeros((data.shape[0], gen.N), dtype=float)
        dC_kp1_q_ref = numpy.zeros((data.shape[0], gen.N), dtype=float)

        dC_kp1_x_ref[50:,:] = 0.2 #m/s
        dC_kp1_y_ref[50:,:] = 0.2 #m/s

        gen.dC_kp1_x_ref[:] = dC_kp1_x_ref[0,:]
        gen.dC_kp1_y_ref[:] = dC_kp1_y_ref[0,:]

        gen.solve()

        for i in range(100):
            print 'iteration: i = ', i
            gen.dC_kp1_x_ref[:] = dC_kp1_x_ref[i,:]
            gen.dC_kp1_y_ref[:] = dC_kp1_y_ref[i,:]

            gen.c_k_x[0] = gen.  C_kp1_x[0]
            gen.c_k_x[1] = gen. dC_kp1_x[0]
            gen.c_k_x[2] = gen.ddC_kp1_x[0]

            gen.c_k_y[0] = gen.  C_kp1_y[0]
            gen.c_k_y[1] = gen. dC_kp1_y[0]
            gen.c_k_y[2] = gen.ddC_kp1_y[0]

            gen.c_k_q[0] = gen.  C_kp1_q[0]
            gen.c_k_q[1] = gen. dC_kp1_q[0]
            gen.c_k_q[2] = gen.ddC_kp1_q[0]

            gen.update()
            gen.solve()

            # get reference from data
            dddC_k_x = data[i,  0:16]
            dddC_k_y = data[i, 18:34]
            F_k_x    = data[i, 16:18]
            F_k_y    = data[i, 34:36]

            assert_allclose(gen.dddC_k_x, dddC_k_x, rtol=RTOL, atol=ATOL)
            assert_allclose(gen.dddC_k_y, dddC_k_y, rtol=RTOL, atol=ATOL)
            assert_allclose(gen.dddC_k_y, 0.0, rtol=RTOL, atol=ATOL)

            assert_allclose(gen.F_k_x, F_k_x, rtol=RTOL, atol=ATOL)
            assert_allclose(gen.F_k_y, F_k_y, rtol=RTOL, atol=ATOL)
            assert_allclose(gen.F_k_q, 0.0, rtol=RTOL, atol=ATOL)


if __name__ == '__main__':
    try:
        import nose
        nose.runmodule()
    except ImportError:
        err_str = 'nose needed for unittests.\nPlease install using:\n   sudo pip install nose'
        raise ImportError(err_str)
