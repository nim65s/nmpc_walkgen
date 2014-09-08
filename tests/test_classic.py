import os
import numpy
from numpy.testing import *

from walking_generator.classic import ClassicGenerator
try:
    from qpoases import PyOptions as Options
    from qpoases import PyPrintLevel as PrintLevel
    from qpoases import PySQProblem as SQProblem
    from qpoases import PySolutionAnalysis as SolutionAnalysis
except ImportError:
    err_str = 'Please install qpOASES python interface, else you will not be able to use this pattern generator.'
    raise ImportError(err_str)

BASEDIR = os.path.dirname(os.path.abspath(__file__))

class TestClassecGenerator(TestCase):
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

        gen.ori_A[...]   = numpy.eye(gen.ori_nc, gen.ori_nc) + numpy.eye(gen.ori_nc, gen.ori_nv, k=1)
        gen.ori_lbA[...] = -numpy.ones((gen.ori_nc,))
        gen.ori_ubA[...] =  numpy.ones((gen.ori_nc,))

        # define solution
        ori_x = -numpy.ones((gen.ori_nv,))*0.11764706
        ori_x[gen.ori_nc] = 0.5
        pos_x = -numpy.ones((gen.pos_nv,))*0.5
        pos_f = -13.5
        ori_f = -1.2573529411764695

        # test first qp solution
        gen._solve_qp()
        gen._postprocess_solution()

        # get solution
        assert_allclose(gen.ori_dofs, ori_x)
        assert_allclose(gen.ori_qp.getObjVal(), ori_f)
        assert_allclose(gen.pos_dofs, pos_x)
        assert_allclose(gen.pos_qp.getObjVal(), pos_f)

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

        assert_allclose(gen.pos_dofs, x)
        assert_allclose(gen.pos_qp.getObjVal(), f)
        assert_allclose(gen.ori_dofs, x)
        assert_allclose(gen.ori_qp.getObjVal(), f)

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

        assert_allclose(gen.pos_dofs, x)
        assert_allclose(gen.pos_qp.getObjVal(), f)
        assert_allclose(gen.ori_dofs, x)
        assert_allclose(gen.ori_qp.getObjVal(), f)

    def test_qp_matrix_setup_against_real_pattern_generator(self):
        pos_Q = numpy.loadtxt(os.path.join(BASEDIR, "data", "Q.dat"))
        pos_P = numpy.loadtxt(os.path.join(BASEDIR, "data", "P.dat"))

        # instantiate pattern generator
        gen = ClassicGenerator()

        # setup QP matrices
        gen._preprocess_solution()

        assert_allclose(gen.pos_H, pos_Q)
        assert_allclose(gen.pos_g, pos_P)

    def test_generator_with_zero_reference_velocity(self):
        gen = ClassicGenerator()

        # set reference velocities to zero
        gen.dC_kp1_x_ref[...] = 0.0
        gen.dC_kp1_y_ref[...] = 0.0
        gen.dC_kp1_q_ref[...] = 0.0

        gen.solve()

        for i in range(100):
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

            assert_allclose(gen.dddC_k_x, 0.0)
            assert_allclose(gen.dddC_k_y, 0.0)
            assert_allclose(gen.dddC_k_q, 0.0)

            assert_allclose(gen.F_k_x, 0.0)
            assert_allclose(gen.F_k_y, 0.0)
            assert_allclose(gen.F_k_q, 0.0)

    def test_against_real_pattern_genererator_walkForward2m_s(self):
        # get test data
        data = numpy.loadtxt(os.path.join(BASEDIR, "data",
            "walkForward2m_s.dat")
        )

        #
        gen = ClassicGenerator()

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

            assert_allclose(gen.dddC_k_x, dddC_k_x)
            assert_allclose(gen.dddC_k_y, dddC_k_y)
            assert_allclose(gen.dddC_k_y, 0.0)

            assert_allclose(gen.F_k_x, F_k_x)
            assert_allclose(gen.F_k_y, F_k_y)
            assert_allclose(gen.F_k_q, 0.0)

    def test_against_real_pattern_genererator_walk_forward_interpolation(self):
         # get test data
         data = numpy.loadtxt(os.path.join(BASEDIR, "data",
             "walkForward2m_sInterpolation.dat")
         )

         time   = data[:, 0]
         C_x    = data[:, 1]
         C_y    = data[:, 2]
         C_z    = data[:, 3]
         C_q    = data[:, 4]
         dC_x   = data[:, 5]
         dC_y   = data[:, 6]
         dC_z   = data[:, 7]
         Z_x    = data[:, 8]
         Z_y    = data[:, 9]
         Fl_x   = data[:,10]
         Fl_y   = data[:,11]
         Fl_z   = data[:,12]
         dFl_x  = data[:,13]
         dFl_y  = data[:,14]
         dFl_z  = data[:,15]
         ddFl_x = data[:,16]
         ddFl_y = data[:,17]
         ddFl_z = data[:,18]
         Fl_q   = data[:,19]
         dFl_q  = data[:,20]
         ddFl_q = data[:,21]
         Fr_x   = data[:,10]
         Fr_y   = data[:,11]
         Fr_z   = data[:,12]
         dFr_x  = data[:,13]
         dFr_y  = data[:,14]
         dFr_z  = data[:,15]
         ddFr_x = data[:,16]
         ddFr_y = data[:,17]
         ddFr_z = data[:,18]
         Fr_q   = data[:,19]
         dFr_q  = data[:,20]
         ddFr_q = data[:,21]

         gen = ClassicGenerator()

    def test_against_real_pattern_genererator_walkSideward2m_s(self):
        # get test data
        data = numpy.loadtxt(os.path.join(BASEDIR, "data",
            "walkSideward2m_s.dat")
        )

    def test_against_real_pattern_genererator_walk_sideward_interpolation(self):
        # get test data
        data = numpy.loadtxt(os.path.join(BASEDIR, "data",
            "walkSideward2m_sInterpolation.dat")
        )

        time   = data[:, 0]
        C_x    = data[:, 1]
        C_y    = data[:, 2]
        C_z    = data[:, 3]
        C_q    = data[:, 4]
        dC_x   = data[:, 5]
        dC_y   = data[:, 6]
        dC_z   = data[:, 7]
        Z_x    = data[:, 8]
        Z_y    = data[:, 9]
        Fl_x   = data[:,10]
        Fl_y   = data[:,11]
        Fl_z   = data[:,12]
        dFl_x  = data[:,13]
        dFl_y  = data[:,14]
        dFl_z  = data[:,15]
        ddFl_x = data[:,16]
        ddFl_y = data[:,17]
        ddFl_z = data[:,18]
        Fl_q   = data[:,19]
        dFl_q  = data[:,20]
        ddFl_q = data[:,21]
        Fr_x   = data[:,10]
        Fr_y   = data[:,11]
        Fr_z   = data[:,12]
        dFr_x  = data[:,13]
        dFr_y  = data[:,14]
        dFr_z  = data[:,15]
        ddFr_x = data[:,16]
        ddFr_y = data[:,17]
        ddFr_z = data[:,18]
        Fr_q   = data[:,19]
        dFr_q  = data[:,20]
        ddFr_q = data[:,21]


if __name__ == '__main__':
    try:
        import nose
        nose.runmodule()
    except ImportError:
        err_str = 'nose needed for unittests.\nPlease install using:\n   sudo pip install nose'
        raise ImportError(err_str)
