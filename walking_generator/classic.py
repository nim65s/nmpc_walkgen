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


class ClassicGenerator(BaseGenerator):
    """
    Reimplementation of current state-of-the-art pattern
    generator for HRP-2 of CNRS-LAAS, Toulouse.

    Solve QPs for position and orientation of CoM and feet
    independently of each other in each timestep.
    First solve  for orientations, then solve for the postions.
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
        super(ClassicGenerator, self).__init__(
            N, T, T_step, fsm_state, fsm_sl
        )
        # TODO for speed up one can define members of BaseGenerator as
        #      direct views of QP data structures according to walking report
        #      Maybe do that later!

        # The pattern generator has to solve the following kind of
        # problem in each iteration

        # min_x 1/2 * x^T * H(w0) * x + x^T g(w0)
        # s.t.   lbA(w0) <= A(w0) * x <= ubA(w0)
        #         lb(w0) <=         x <= ub(wo)

        # Because of varying H and A, we have to use the
        # SQPProblem class, which supports this kind of QPs

        # rename for convenience
        N  = self.N
        nf = self.nf

        # define some qpOASES specific things
        self.cpu_time = 0.1 # upper bound on CPU time, 0 is no upper limit
        self.nwsr     = 100      # number of working set recalculations
        self.options = Options()
        self.options.setToMPC()
        #self.options.printLevel = PrintLevel.LOW

        # FOR ORIENTATIONS
        # define dimensions
        self.ori_nv = 2*self.N
        self.ori_nc = (
              self.nc_fvel_eq
            + self.nc_fpos_ineq
            + self.nc_fvel_ineq
        )

        # setup problem
        self.ori_dofs = numpy.zeros(self.ori_nv)
        self.ori_qp = SQProblem(self.ori_nv, self.ori_nc)
        self.ori_qp.setOptions(self.options) # load NMPC options

        self.ori_H   =  numpy.zeros((self.ori_nv,self.ori_nv))
        self.ori_A   =  numpy.zeros((self.ori_nc,self.ori_nv))
        self.ori_g   =  numpy.zeros((self.ori_nv,))
        self.ori_lb  = -numpy.ones((self.ori_nv,))*1e+08
        self.ori_ub  =  numpy.ones((self.ori_nv,))*1e+08
        self.ori_lbA = -numpy.ones((self.ori_nc,))*1e+08
        self.ori_ubA =  numpy.ones((self.ori_nc,))*1e+08

        self._ori_qp_is_initialized = False

        # save computation time and working set recalculations
        self.ori_qp_nwsr    = 0.0
        self.ori_qp_cputime = 0.0

        # FOR POSITIONS
        # define dimensions
        self.pos_nv = 2*(self.N + self.nf)
        self.pos_nc = (
            self.nc_cop
            + self.nc_foot_position
            + self.nc_fchange_eq
        )

        # setup problem
        self.pos_dofs = numpy.zeros(self.pos_nv)
        self.pos_qp = SQProblem(self.pos_nv, self.pos_nc)
        self.pos_qp.setOptions(self.options)

        self.pos_H   = numpy.zeros((self.pos_nv,self.pos_nv))
        self.pos_A   = numpy.zeros((self.pos_nc,self.pos_nv))
        self.pos_g   = numpy.zeros((self.pos_nv,))
        self.pos_lb  = -numpy.ones((self.pos_nv,))*1e+08
        self.pos_ub  =  numpy.ones((self.pos_nv,))*1e+08
        self.pos_lbA = -numpy.ones((self.pos_nc,))*1e+08
        self.pos_ubA =  numpy.ones((self.pos_nc,))*1e+08

        self._pos_qp_is_initialized = False

        # save computation time and working set recalculations
        self.pos_qp_nwsr    = 0.0
        self.pos_qp_cputime = 0.0

        # dummy matrices
        self._ori_Q = numpy.zeros((2*self.N, 2*self.N))
        self._ori_p = numpy.zeros((2*self.N,))
        self._pos_Q = numpy.zeros((self.N + self.nf, self.N + self.nf))
        self._pos_p = numpy.zeros((self.N + self.nf,))

        # add additional keys that should be saved
        self._data_keys.append('ori_qp_nwsr')
        self._data_keys.append('ori_qp_cputime')
        self._data_keys.append('pos_qp_nwsr')
        self._data_keys.append('pos_qp_cputime')

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

        # ORIENTATIONS

        # initialize with actual values, else take last known solution
        # NOTE for warmstart last solution is taken from qpOASES internal memory
        if not self._ori_qp_is_initialized:
            # ori_dofs = ( dddF_k_qR )
            #            ( dddF_k_qL )

            self.ori_dofs[:N] = self.dddF_k_qR
            self.ori_dofs[N:] = self.dddF_k_qL

            # TODO guess initial active set
            # this requires changes to the python interface

        # define QP matrices

        # H = ( Q_k_q )
        self._update_ori_Q() # updates values in _Q
        self.ori_H  [:,:] = self._ori_Q

        # g = ( p_k_q )
        self._update_ori_p() # updates values in _p
        self.ori_g  [:]   = self._ori_p

        # ORIENTATION LINEAR CONSTRAINTS
        # velocity constraints on support foot to freeze movement
        a = 0
        b = self.nc_fvel_eq
        self.ori_A  [a:b] = self.A_fvel_eq
        self.ori_lbA[a:b] = self.B_fvel_eq
        self.ori_ubA[a:b] = self.B_fvel_eq

        # box constraints for maximum orientation change
        a = self.nc_fvel_eq
        b = self.nc_fvel_eq + self.nc_fpos_ineq
        self.ori_A  [a:b] = self.A_fpos_ineq
        self.ori_lbA[a:b] = self.lbB_fpos_ineq
        self.ori_ubA[a:b] = self.ubB_fpos_ineq

        # box constraints for maximum angular velocity
        a = self.nc_fvel_eq + self.nc_fpos_ineq
        b = self.nc_fvel_eq + self.nc_fpos_ineq + self.nc_fvel_ineq
        self.ori_A  [a:b] = self.A_fvel_ineq
        self.ori_lbA[a:b] = self.lbB_fvel_ineq
        self.ori_ubA[a:b] = self.ubB_fvel_ineq

        # ORIENTATION BOX CONSTRAINTS
        #self.ori_lb [...] = 0.0
        #self.ori_ub [...] = 0.0

        # POSITIONS

        # initialize with actual values, else take last known solution
        # NOTE for warmstart last solution is taken from qpOASES internal memory
        if not self._pos_qp_is_initialized:
            # pos_dofs = ( dddC_kp1_x )
            #            (      F_k_x )
            #            ( dddC_kp1_y )
            #            (      F_k_y )

            self.pos_dofs[0  :0+N   ] = self.dddC_k_x
            self.pos_dofs[0+N:0+N+nf] = self.F_k_x
            a = N+nf
            self.pos_dofs[a  :a+N   ] = self.dddC_k_y
            self.pos_dofs[a+N:a+N+nf] = self.F_k_y

            # TODO guess initial active set
            # this requires changes to the python interface

        # define QP matrices

        # H = ( Q_k   0 )
        #     (   0 Q_k )
        self._update_pos_Q() # updates values in _Q
        self.pos_H  [ :N+nf,  :N+nf] = self._pos_Q
        self.pos_H  [-N-nf:, -N-nf:] = self._pos_Q

        # g = ( p_k_x )
        #     ( p_k_y )
        self._update_pos_p('x') # updates values in _p
        self.pos_g  [ :N+nf] = self._pos_p
        self._update_pos_p('y') # updates values in _p
        self.pos_g  [-N-nf:] = self._pos_p

        # lbA <= A x <= ubA
        # (       ) <= (    Acop ) <= (    Bcop )
        # (eqBfoot) <= ( eqAfoot ) <= ( eqBfoot )
        # (       ) <= (   Afoot ) <= (   Bfoot )

        # CoP constraints
        a = 0
        b = self.nc_cop
        self.pos_A  [a:b] = self.Acop
        self.pos_lbA[a:b] = self.lbBcop
        self.pos_ubA[a:b] = self.ubBcop

        #foot inequality constraints
        a = self.nc_cop
        b = self.nc_cop + self.nc_foot_position
        self.pos_A  [a:b] = self.Afoot
        self.pos_lbA[a:b] = self.lbBfoot
        self.pos_ubA[a:b] = self.ubBfoot

        #foot equality constraints
        a = self.nc_cop + self.nc_foot_position
        b = self.nc_cop + self.nc_foot_position + self.nc_fchange_eq
        self.pos_A  [a:b] = self.eqAfoot
        self.pos_lbA[a:b] = self.eqBfoot
        self.pos_ubA[a:b] = self.eqBfoot

        # NOTE: they stay plus minus infinity, i.e. 1e+08
        #self.pos_lb [...] = 0.0
        #self.pos_ub [...] = 0.0

    def _update_ori_Q(self):
        '''
        Update Hessian block Q according to walking report

        min a/2 || dC_kp1_q_ref - dF_k_q ||_2^2
        Q = ( QR  0 )
            (  0 QL )
        QR = ( a * Pvu^T * E_FR^T * E_FR * Pvu )
        '''
        # rename for convenience
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

        Pvu = self.Pvu

        # assemble Hessian matrix
        # QR = ( a * Pvu^T * E_FR^T * E_FR * Pvu )
        a = 0; b = N
        c = 0; d = N
        QR = self._ori_Q[a:b,c:d]
        QR[...] = alpha * Pvu.transpose().dot(E_FR.transpose()).dot(E_FR).dot(Pvu)

        # QL = ( a * Pvu^T * E_FL^T * E_FL * Pvu )
        # Q = ( * , * )
        #     ( * ,[*])
        a = N; b = 2*N
        c = N; d = 2*N
        QL = self._ori_Q[a:b,c:d]
        QL[...] = alpha * Pvu.transpose().dot(E_FL.transpose()).dot(E_FL).dot(Pvu)

    def _update_ori_p(self):
        """
        Update pass gradient block p according to walking report

        min a/2 || dC_kp1_q_ref - dF_k_q ||_2^2
        p = ( pR )
            ( pL )
        pR = ( a * Pvu^T * E_FR^T * (E_FR * Pvs * f_k_qR - dC_kp1_q_ref[N/2] )
        """
        # rename for convenience
        N  = self.N
        nf = self.nf

        # weights
        alpha = self.a
        beta  = self.b
        gamma = self.c

        # matrices
        E_FR = self.E_FR
        E_FL = self.E_FL

        f_k_qR = self.f_k_qR
        f_k_qL = self.f_k_qL

        Pvs   = self.Pvs
        Pvu   = self.Pvu

        dC_kp1_q_ref = self.dC_kp1_q_ref

        # pR = ( a * Pvu^T * E_FL^T * (E_FL * Pvs * f_k_qL + dC_kp1_q_ref) )
        # p = ([*]) =
        #     ( * )
        a = 0; b = N
        self._ori_p[a:b] = \
            alpha * Pvu.transpose().dot(E_FR.transpose()).dot(E_FR.dot(Pvs).dot(f_k_qR) - dC_kp1_q_ref)

        # p = ( * ) =
        #     ([*])
        a = N; b = 2*N
        self._ori_p[a:b] = \
            alpha * Pvu.transpose().dot(E_FL.transpose()).dot(E_FL.dot(Pvs).dot(f_k_qL) - dC_kp1_q_ref)

    def _update_pos_Q(self):
        '''
        Update Hessian block Q according to walking report

        Q = ( a*Pvu*Pvu + b*Ppu*E*T*E*Ppu + c*Pzu*Pzu + d*I, -c*Pzu*V_kp1  )
            (                                  -c*Pzu*V_kp1, c*V_kp1*V_kp1 )
        '''
        # rename for convenience
        N  = self.N
        nf = self.nf

        # weights
        alpha = self.a
        beta  = self.b
        gamma = self.c
        delta = self.d

        # cast matrices for convenience
        Ppu   = numpy.asmatrix(self.Ppu)
        Pvu   = numpy.asmatrix(self.Pvu)
        Pzu   = numpy.asmatrix(self.Pzu)
        V_kp1 = numpy.asmatrix(self.V_kp1)

        # Q = ([*], * ) = a*Pvu*Pvu + b*Ppu*E*E*Ppu + c*Pzu*Pzu + d*I
        #     ( * , * )
        a = 0; b = N
        c = 0; d = N
        self._pos_Q[a:b,c:d] = alpha * Pvu.transpose() * Pvu \
                         + gamma * Pzu.transpose() * Pzu \
                         + delta * numpy.eye(N)

        # Q = ( * ,[*])
        #     ( * , * )
        a = 0; b = N
        c = N; d = N+nf
        self._pos_Q[a:b,c:d] = -gamma * Pzu.transpose() * V_kp1

        # Q = (  * , * ) = ( * , [*] )^T
        #     ( [*], * )   ( * ,  *  )
        dummy = self._pos_Q[a:b,c:d]
        a = N; b = N+nf
        c = 0; d = N
        self._pos_Q[a:b,c:d] = dummy.transpose()

        # Q = ( * , * )
        #     ( * ,[*])
        a = N; b = N+nf
        c = N; d = N+nf
        self._pos_Q[a:b,c:d] = gamma * V_kp1.transpose() * V_kp1

    def _update_pos_p(self, case=None):
        """
        Update pass gradient block p according to walking report

        p = ( a*Pvu*(Pvs*ck - Refk+1) + b*Ppu*E*(E*Pps*cx - Refk+1) + c*Pzu*(Pzs*ck - vk+1*fk )
            (                                                       -c*Vk+1*(Pzs*ck - vk+1*fk )
        """
        if case == 'x':
            f_k = self.f_k_x

            c_k        = utility.cast_array_as_matrix(self.c_k_x)
            dC_kp1_ref = utility.cast_array_as_matrix(self.dC_kp1_x_ref)

        elif case == 'y':
            f_k = self.f_k_y

            c_k        = utility.cast_array_as_matrix(self.c_k_y)
            dC_kp1_ref = utility.cast_array_as_matrix(self.dC_kp1_y_ref)
        else:
            err_str = 'Please use either case "x" or "y" for this routine'
            raise AttributeError(err_str)

        # rename for convenience
        N  = self.N
        nf = self.nf

        # weights
        alpha = self.a
        beta  = self.b
        gamma = self.c

        # matrices
        v_kp1 = utility.cast_array_as_matrix(self.v_kp1)

        Pvs   = numpy.asmatrix(self.Pvs)
        Pvu   = numpy.asmatrix(self.Pvu)
        Pps   = numpy.asmatrix(self.Pps)
        Ppu   = numpy.asmatrix(self.Ppu)
        Pzs   = numpy.asmatrix(self.Pzs)
        Pzu   = numpy.asmatrix(self.Pzu)
        V_kp1 = numpy.asmatrix(self.V_kp1)

        # p = ([*]) =
        #     ( * )
        a = 0; b = N
        self._pos_p[a:b] = (
              alpha * Pvu.transpose() *(Pvs*c_k - dC_kp1_ref)
            + gamma * Pzu.transpose() *(Pzs*c_k - v_kp1*f_k)
            #+ b*Ppu.transpose() * E.transpose() * E * Ppu \
        ).ravel()

        # p = ( * ) =
        #     ([*])
        a = N; b = N+nf
        self._pos_p[a:b] = (
            -gamma * V_kp1.transpose() * (Pzs*c_k - v_kp1*f_k)
        ).ravel()

    def _solve_qp(self):
        """
        Solve QP first run with init functionality and other runs with warmstart
        """
        #sys.stdout.write('Solve for orientations:\n')
        if not self._ori_qp_is_initialized:
            ret, nwsr, cputime = self.ori_qp.init(
                self.ori_H, self.ori_g, self.ori_A,
                self.ori_lb, self.ori_ub,
                self.ori_lbA, self.ori_ubA,
                self.nwsr, self.cpu_time
            )
            self._ori_qp_is_initialized = True
        else:
            ret, nwsr, cputime = self.ori_qp.hotstart(
                self.ori_H, self.ori_g, self.ori_A,
                self.ori_lb, self.ori_ub,
                self.ori_lbA, self.ori_ubA,
                self.nwsr, self.cpu_time
            )

        # orientation primal solution
        self.ori_qp.getPrimalSolution(self.ori_dofs)

        # save qp solver data
        self.ori_qp_nwsr    = nwsr          # working set recalculations
        self.ori_qp_cputime = cputime*1000. # in milliseconds

        #sys.stdout.write('Solve for positions:\n')
        if not self._pos_qp_is_initialized:
            ret, nwsr, cputime = self.pos_qp.init(
                self.pos_H, self.pos_g, self.pos_A,
                self.pos_lb, self.pos_ub,
                self.pos_lbA, self.pos_ubA,
                self.nwsr, self.cpu_time
            )
            self._pos_qp_is_initialized = True
        else:
            ret, nwsr, cputime = self.pos_qp.hotstart(
                self.pos_H, self.pos_g, self.pos_A,
                self.pos_lb, self.pos_ub,
                self.pos_lbA, self.pos_ubA,
                self.nwsr, self.cpu_time
            )

        # position primal solution
        self.pos_qp.getPrimalSolution(self.pos_dofs)

        # save qp solver data
        self.pos_qp_nwsr    = nwsr          # working set recalculations
        self.pos_qp_cputime = cputime*1000. # in milliseconds

    def _postprocess_solution(self):
        """ Get solution and put it back into generator data structures """
        # rename for convenience
        N  = self.N
        nf = self.nf

        # extract dofs
        # ori_dofs = ( dddF_k_qR )
        #            ( dddF_k_qL )

        self.dddF_k_qR[:] = self.ori_dofs[:N]
        self.dddF_k_qL[:] = self.ori_dofs[N:]

        # extract dofs
        # pos_dofs = ( dddC_kp1_x )
        #            (      F_k_x )
        #            ( dddC_kp1_y )
        #            (      F_k_y )
        self.dddC_k_x[:] = self.pos_dofs[0  :0+N   ]
        self.F_k_x[:]    = self.pos_dofs[0+N:0+N+nf]
        a = N + nf
        self.dddC_k_y[:] = self.pos_dofs[a  :a+N   ]
        self.F_k_y[:]    = self.pos_dofs[a+N:a+N+nf]
