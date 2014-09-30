import sys
import numpy
import utility

from base import BaseGenerator
from visualization import PlotData

# Try to get qpOASES SQP Problem class
try:
    from qpoases import PyOptions as Options
    from qpoases import PyPrintLevel as PrintLevel
    from qpoases import PySQProblem as SQProblem
    from qpoases import PySolutionAnalysis as SolutionAnalysis
except ImportError:
    err_str = 'Please install qpOASES python interface, else you will not be able to use this pattern generator.'
    raise ImportError(err_str)

class NMPCGenerator(BaseGenerator):
    """
    Implementation of the combined problems using NMPC techniques.

    Solve QP for position and orientation of CoM and feet simultaneously in
    each timestep. Calculates derivatives and updates states in each step.
    """
    def __init__(
        self, N=16, T=0.1, T_step=0.8,
        fsm_state='D', fsm_sl=1
    ):
        """
        Initialize pattern generator matrices through base class
        and allocate two QPs one for optimzation of orientation and
        one for position of CoM and feet.

        """
        super(NMPCGenerator, self).__init__(
            N, T, T_step, fsm_state, fsm_sl
        )
        # The pattern generator has to solve the following kind of
        # problem in each iteration

        # min_x 1/2 * x.T * H(w0) * x + x.T g(w0)
        # s.t.   lbA(w0) <= A(w0) * x <= ubA(w0)
        #         lb(w0) <=         x <= ub(wo)

        # Because of varying H and A, we have to use the
        # SQPProblem class, which supports this kind of QPs

        # rename for convenience
        N  = self.N
        nf = self.nf

        # define some qpOASES specific things
        self.cpu_time = 0.1 # upper bound on CPU time, 0 is no upper limit
        self.nwsr     = 100 # # of working set recalculations
        self.options = Options()
        self.options.setToMPC()
        self.options.printLevel = PrintLevel.LOW

        # define variable dimensions
        # variables of:     position + orientation
        self.nv = 2*(self.N+self.nf) + 2*N

        # define constraint dimensions
        self.nc = 0#self.fvel# + self.facc

        # setup problem
        self.dofs = numpy.zeros(self.nv)
        self.qp   = SQProblem(self.nv, self.nc)

        # load NMPC options
        self.qp.setOptions(self.options)

        self.qp_H   =  numpy.eye(self.nv,self.nv)
        self.qp_A   =  numpy.zeros((self.nc,self.nv))
        self.qp_g   =  numpy.zeros((self.nv,))
        self.qp_lb  = -numpy.ones((self.nv,))*1e+08
        self.qp_ub  =  numpy.ones((self.nv,))*1e+08
        self.qp_lbA = -numpy.ones((self.nc,))*1e+08
        self.qp_ubA =  numpy.ones((self.nc,))*1e+08

        self._qp_is_initialized = False

        # save computation time and working set recalculations
        self.qp_nwsr    = 0.0
        self.qp_cputime = 0.0

        # setup analyzer for solution analysis
        analyser = SolutionAnalysis()

        # helper matrices for common expressions
        self.Hx     = numpy.zeros((1, 2*(N+nf)), dtype=float)
        self.Q_k_x  = numpy.zeros((N+nf, N+nf),  dtype=float)
        self.p_k_x  = numpy.zeros((N+nf,),       dtype=float)
        self.p_k_y  = numpy.zeros((N+nf,),       dtype=float)

        self.Hq     = numpy.zeros((1, 2*N), dtype=float)
        self.Q_k_qR = numpy.zeros((N, N),   dtype=float)
        self.Q_k_qL = numpy.zeros((N, N),   dtype=float)
        self.p_k_qR = numpy.zeros((N),      dtype=float)
        self.p_k_qL = numpy.zeros((N),      dtype=float)

        # add additional keys that should be saved
        self._data_keys.append('qp_nwsr')
        self._data_keys.append('qp_cputime')

        # reinitialize plot data structure
        self.data = PlotData(self)

    def solve(self):
        """ Process and solve problem, s.t. pattern generator data is consistent """
        self._preprocess_solution()
        self._solve_qp()
        self._postprocess_solution()

    def _preprocess_solution(self):
        """ Update matrices and get them into the QP data structures """
        # rename for convenience
        N  = self.N
        nf = self.nf

        # dofs
        dofs = self.dofs

        dddC_k_x  = self.dddC_k_x
        F_k_x     = self.F_k_x
        dddC_k_y  = self.dddC_k_y
        F_k_y     = self.F_k_y
        dddF_k_qR = self.dddF_k_qR
        dddF_k_qL = self.dddF_k_qL

        # inject dofs for convenience
        # dofs = ( dddC_k_x ) N
        #        (    F_k_x ) nf
        #        ( dddC_k_y ) N
        #        (    F_k_y ) nf
        #        ( dddF_k_q ) N
        #        ( dddF_k_q ) N
        dofs[0  :0+N   ] = dddC_k_x
        dofs[0+N:0+N+nf] = F_k_x

        a = N+nf
        dofs[a  :a+N   ] = dddC_k_y
        dofs[a+N:a+N+nf] = F_k_y

        a = 2*(N+nf)
        dofs[  a:a+N]    = dddF_k_qR
        dofs[ -N:]       = dddF_k_qL

        # define position and orientation dofs
        # U_k = ( U_k_xy, U_k_q).T
        # U_k_xy = ( dddC_k_x ) N
        #          (    F_k_x ) nf
        #          ( dddC_k_y ) N
        #          (    F_k_y ) nf
        # U_k_q  = ( dddF_k_q ) N
        #          ( dddF_k_q ) N

        # position dofs
        U_k    = self.dofs
        U_k_xy = U_k[    :2*(N+nf)]
        U_k_x  = U_k_xy[:(N+nf)]
        U_k_y  = U_k_xy[(N+nf):]

        # orientation dofs
        U_k_q   = U_k  [-2*N: ]
        U_k_qR  = U_k_q[    :N]
        U_k_qL  = U_k_q[   N: ]

        # position dimensions
        nU_k    = U_k.shape[0]
        nU_k_xy = U_k_xy.shape[0]
        nU_k_x  = U_k_x.shape[0]
        nU_k_y  = U_k_y.shape[0]

        # orientation dimensions
        nU_k_q  = U_k_q.shape[0]
        nU_k_qR = U_k_qR.shape[0]
        nU_k_qL = U_k_qL.shape[0]

        # initialize with actual values, else take last known solution
        # NOTE for warmstart last solution is taken from qpOASES internal memory
        if not self._qp_is_initialized:
            # TODO guess initial active set
            # this requires changes to the python interface
            pass

        # calculate some common sub expressions
        self._calculate_common_expressions()

        # POSITION QP
        # rename matrices
        Q_k_x = self.Q_k_x
        Q_k_y = self.Q_k_x # NOTE it's exactly the same!
        p_k_x = self.p_k_x
        p_k_y = self.p_k_y

        # ORIENTATION QP
        # rename matrices
        Q_k_qR = self.Q_k_qR
        Q_k_qL = self.Q_k_qL
        p_k_qR = self.p_k_qR
        p_k_qL = self.p_k_qL

        # define QP matrices
        # Gauss-Newton Hessian approximation
        # define sub blocks
        # H = (Hxy | Hqx)
        #     (Hxq | Hqq)
        Hxx = self.qp_H[:nU_k_xy,:nU_k_xy]
        Hxq = self.qp_H[:nU_k_xy,nU_k_xy:]
        Hqx = self.qp_H[nU_k_xy:,:nU_k_xy]
        Hqq = self.qp_H[nU_k_xy:,nU_k_xy:]

        # Hx = ( U_k_xy.T Q_k_xy + p_k_xy )
        #    = ( U_k_x.T Q_k_x + p_k_x | U_k_y.T Q_k_y + p_k_y)
        Hx = self.Hx
        Hx[:, :nU_k_x] = 0.5 * U_k_x.transpose().dot(Q_k_x) + p_k_x
        Hx[:,-nU_k_x:] = 0.5 * U_k_y.transpose().dot(Q_k_y) + p_k_y

        # Hq = ( U_k_q.T Q_k_q + p_k_q )
        #    = ( U_k_qR.T Q_k_qR + p_k_qR | U_k_qL.T Q_k_qL + p_k_qL)
        Hq = self.Hq
        Hq[:, :nU_k_qR] = 0.5 * U_k_qR.transpose().dot(Q_k_qR) + p_k_qR
        Hq[:,-nU_k_qL:] = 0.5 * U_k_qL.transpose().dot(Q_k_qL) + p_k_qL

        # fill matrices
        # Hxx = Hx.T * Hx
        # Hxq = Hx.T * Hq
        # Hqx = Hq.T * Hx
        # Hqq = Hq.T * Hq
        Hxx[...] = Hx.transpose().dot(Hx)
        Hxq[...] = Hx.transpose().dot(Hq)
        Hqx[...] = Hxq.transpose()
        Hqq[...] = Hq.transpose().dot(Hq)

        # Gradient of Objective
        # define sub blocks
        # g = (gx)
        #     (gq)
        gx = self.qp_g[:nU_k_xy]
        gq = self.qp_g[-nU_k_q:]

        # gx = ( U_k_x.T Q_k_x + p_k_x )
        gx[...] = Hx

        # gq = ( U_k_q.T Q_k_q + p_k_q )
        gq[...] = Hq

        """
        # ORIENTATION LINEAR CONSTRAINTS
        # velocity constraints on support foot to freeze movement
        a = 0
        b = self.ori_fvel_eq
        self.ori_A  [a:b] = self.A_fvel_eq
        self.ori_lbA[a:b] = self.B_fvel_eq
        self.ori_ubA[a:b] = self.B_fvel_eq

        # box constraints for maximum orientation change
        a = self.ori_fvel_eq
        b = self.ori_fvel_eq + self.ori_fpos_ineq
        self.ori_A  [a:b] = self.A_fpos_ineq
        self.ori_lbA[a:b] = self.lbB_fpos_ineq
        self.ori_ubA[a:b] = self.ubB_fpos_ineq

        # box constraints for maximum angular velocity
        a = self.ori_fvel_eq + self.ori_fpos_ineq
        b = self.ori_fvel_eq + self.ori_fpos_ineq + self.ori_fvel_ineq
        self.ori_A  [a:b] = self.A_fvel_ineq
        self.ori_lbA[a:b] = self.lbB_fvel_ineq
        self.ori_ubA[a:b] = self.ubB_fvel_ineq
        """

    def _calculate_common_expressions(self):
        """
        encapsulation of complicated matrix assembly of former orientation and
        position QP sub matrices
        """
        #rename for convenience
        N  = self.N
        nf = self.nf

        # weights
        alpha = self.a
        beta  = self.b
        gamma = self.c
        delta = self.d

        # matrices
        E_FR = self.E_FR
        E_FL = self.E_FL

        Pvs = self.Pvs
        Pvu = self.Pvu
        Pzs = self.Pzs
        Pzu = self.Pzu

        c_k_x = self.c_k_x
        c_k_y = self.c_k_y
        f_k_x = self.f_k_x
        f_k_y = self.f_k_y
        f_k_qR = self.f_k_qR
        f_k_qL = self.f_k_qL

        v_kp1 = self.v_kp1
        V_kp1 = self.V_kp1

        dC_kp1_x_ref = self. dC_kp1_x_ref
        dC_kp1_y_ref = self. dC_kp1_y_ref
        dC_kp1_q_ref = self. dC_kp1_q_ref

        # POSITION QP MATRICES
        # Q_k_x = ( Q_k_xXX Q_k_xXF ) = Q_k_y
        #         ( Q_k_xFX Q_k_xFF )
        Q_k_x = self.Q_k_x

        a = 0; b = N
        c = 0; d = N
        Q_k_xXX = Q_k_x[a:b,c:d]

        a = 0; b = N
        c = N; d = N+nf
        Q_k_xXF = Q_k_x[a:b,c:d]

        a = N; b = N+nf
        c = 0; d = N
        Q_k_xFX = Q_k_x[a:b,c:d]

        a = N; b = N+nf
        c = N; d = N+nf
        Q_k_xFF = Q_k_x[a:b,c:d]

        # Q_k_xXX = (  0.5 * a * Pvu^T * Pvu + c * Pzu^T * Pzu + d * I )
        # Q_k_xXF = ( -0.5 * c * Pzu^T * V_kp1 )
        # Q_k_xFX = ( -0.5 * c * Pzu^T * V_kp1 )^T
        # Q_k_xFF = (  0.5 * c * V_kp1^T * V_kp1 )
        Q_k_xXX[...] = (
              alpha * Pvu.transpose().dot(Pvu)
            + gamma * Pzu.transpose().dot(Pzu)
            + delta * numpy.eye(N)
        )
        Q_k_xXF[...] = - gamma * Pzu.transpose().dot(V_kp1)
        Q_k_xFX[...] = Q_k_xXF.transpose()
        Q_k_xFF[...] =   gamma * V_kp1.transpose().dot(V_kp1)

        # p_k_x = ( p_k_xX )
        #         ( p_k_xF )
        p_k_x = self.p_k_x
        p_k_xX = p_k_x[   :N]
        p_k_xF = p_k_x[-nf: ]

        # p_k_xX = (  0.5 * a * Pvu^T * Pvu + c * Pzu^T * Pzu + d * I )
        # p_k_xF = ( -0.5 * c * Pzu^T * V_kp1 )
        p_k_xX[...] = alpha * Pvu.transpose().dot(  Pvs.dot(c_k_x) - dC_kp1_x_ref) \
                    + gamma * Pzu.transpose().dot(  Pzs.dot(c_k_x) - v_kp1.dot(f_k_x))
        p_k_xF[...] =-gamma * V_kp1.transpose().dot(Pzs.dot(c_k_x) - v_kp1.dot(f_k_x))

        # p_k_y = ( p_k_yX )
        #         ( p_k_yF )
        p_k_y = self.p_k_y
        p_k_yX = p_k_y[   :N]
        p_k_yF = p_k_y[-nf: ]

        # p_k_yX = (  0.5 * a * Pvu^T * Pvu + c * Pzu^T * Pzu + d * I )
        # p_k_yF = ( -0.5 * c * Pzu^T * V_kp1 )
        p_k_yX[...] = alpha * Pvu.transpose().dot(  Pvs.dot(c_k_y) - dC_kp1_y_ref) \
                    + gamma * Pzu.transpose().dot(  Pzs.dot(c_k_y) - v_kp1.dot(f_k_y))
        p_k_yF[...] =-gamma * V_kp1.transpose().dot(Pzs.dot(c_k_y) - v_kp1.dot(f_k_y))

        # ORIENTATION QP MATRICES
        # Q_k_qR = ( 0.5 * a * Pvu^T * E_FR^T *  E_FR * Pvu )
        Q_k_qR = self.Q_k_qR
        Q_k_qR[...] = alpha * Pvu.transpose().dot(E_FR.transpose()).dot(E_FR).dot(Pvu)

        # p_k_qR = (       a * Pvu^T * E_FR^T * (E_FR * Pvs * f_k_qR + dC_kp1_q_ref) )
        p_k_qR = self.p_k_qR
        p_k_qR[...] = alpha * Pvu.transpose().dot(E_FR.transpose()).dot(E_FR.dot(Pvs).dot(f_k_qR) - dC_kp1_q_ref)

        # Q_k_qL = ( 0.5 * a * Pvu^T * E_FL^T *  E_FL * Pvu )
        Q_k_qL = self.Q_k_qL
        Q_k_qL[...] = alpha * Pvu.transpose().dot(E_FL.transpose()).dot(E_FL).dot(Pvu)
        # p_k_qL = (       a * Pvu^T * E_FL^T * (E_FL * Pvs * f_k_qL + dC_kp1_q_ref) )
        p_k_qL = self.p_k_qL
        p_k_qL[...] = alpha * Pvu.transpose().dot(E_FL.transpose()).dot(E_FL.dot(Pvs).dot(f_k_qL) - dC_kp1_q_ref)

    def _solve_qp(self):
        """
        Solve QP first run with init functionality and other runs with warmstart
        """
        if not self._qp_is_initialized:
            ret, nwsr, cputime = self.qp.init(
                self.qp_H, self.qp_g, self.qp_A,
                self.qp_lb, self.qp_ub,
                self.qp_lbA, self.qp_ubA,
                self.nwsr, self.cpu_time
            )
            self._qp_is_initialized = True
        else:
            ret, nwsr, cputime = self.qp.hotstart(
                self.qp_H, self.qp_g, self.qp_A,
                self.qp_lb, self.qp_ub,
                self.qp_lbA, self.qp_ubA,
                self.nwsr, self.cpu_time
            )

        # orientation primal solution
        self.qp.getPrimalSolution(self.dofs)

        # save qp solver data
        self.qp_nwsr    = nwsr          # working set recalculations
        self.qp_cputime = cputime*1000. # in milliseconds

    def _postprocess_solution(self):
        """ Get solution and put it back into generator data structures """
        # rename for convenience
        N  = self.N
        nf = self.nf

        # extract dofs
        # dofs = ( dddC_k_x ) N
        #        (    F_k_x ) nf
        #        ( dddC_k_y ) N
        #        (    F_k_y ) nf
        #        ( dddF_k_q ) N
        #        ( dddF_k_q ) N

        # NOTE this time we add an increment to the existing values
        # data(k+1) = data(k) + alpha * dofs

        # TODO add line search when problematic
        alpha = 1.0

        # x values
        self.dddC_k_x[:]  += alpha * self.dofs[0  :0+N   ]
        self.F_k_x[:]     += alpha * self.dofs[0+N:0+N+nf]

        # y values
        a = N + nf
        self.dddC_k_y[:]  += alpha * self.dofs[a  :a+N   ]
        self.F_k_y[:]     += alpha * self.dofs[a+N:a+N+nf]

        # feet orientation
        a =2*(N + nf)
        self.dddF_k_qR[:] += alpha * self.dofs[  a:a+N]
        self.dddF_k_qL[:] += alpha * self.dofs[ -N:]


