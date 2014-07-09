import numpy

class Generator(object):
    """
    Prototype of Walking Pattern Generator, cf.

    TODO cite paper
    *
    *
    *
    """
    # define some constants
    g = 9.81

    def __init__(self, N=15, T=0.1, nf=2, h_com=0.81):
        """
        Initialize pattern generator, i.e.
        * allocate memory for matrices
        * populate them according to walking reference document

        Parameters
        ----------

        N : int
            Number of time steps of prediction horizon

        T : float
            Time increment between two time steps (default: 0.1 [s])

        nf : int
            Number of foot steps planned in advance (default: 2 [footsteps])

        h_com: float
            Height of center of mass for the LIPM (default: 0.81 [m])
        """
        self.N = N
        self.T = T
        self.nf = nf
        self.h_com = h_com

        # qpOASES matrices
        # H = ( Q_x   0 )
        #     (   0 Q_y )
        # g = ( p_x )
        #     ( p_y )
        # xi = ( dddC_kp1_x )
        #      (        f_x )
        #      ( dddC_kp1_y )
        #      (        f_y )

        # TODO for combined problem
        self.NXI = 2*self.N + 2*self.nf
        self.NC  = 0

        self._QP = SQProblem(self.NXI, self.NC)

        self._H = numpy.eye(self.NXI)
        self._g = numpy.zeros(self.NXI)
        self._A = numpy.zeros((self.NC, self.NXI))
        self._c = numpy.zeros(self.NXI)

        self._lbA  = numpy.zeros(self.NC)
        self._ubA  = numpy.zeros(self.NC)

        self._lb = numpy.zeros(self.NXI)
        self._ub = numpy.zeros(self.NXI)

        # flag for first initialization
        self.qp_is_initialized = False

        # number of recalculations of active set
        self.nWSR = 1000

        # maximum number of seconds for QP solving
        self.maxtime = 2

        self.Q = numpy.zeros((self.NXI, self.NXI), dtype=float)
        self.p = numpy.zeros((self.NXI,), dtype=float)
        self.x = numpy.zeros((self.NXI,), dtype=float)

        # center of mass initial values

        self.c_k_x = numpy.zeros((3,), dtype=float)
        self.c_k_y = numpy.zeros((3,), dtype=float)
        self.c_k_q = numpy.zeros((3,), dtype=float)

        # center of mass matrices

        self.  C_kp1_x = numpy.zeros((N,), dtype=float)
        self. dC_kp1_x = numpy.zeros((N,), dtype=float)
        self.ddC_kp1_x = numpy.zeros((N,), dtype=float)

        self.  C_kp1_y = numpy.zeros((N,), dtype=float)
        self. dC_kp1_y = numpy.zeros((N,), dtype=float)
        self.ddC_kp1_y = numpy.zeros((N,), dtype=float)

        self.  C_kp1_q = numpy.zeros((N,), dtype=float)
        self. dC_kp1_q = numpy.zeros((N,), dtype=float)
        self.ddC_kp1_q = numpy.zeros((N,), dtype=float)

        # jerk controls for center of mass

        self.dddC_k_x = numpy.zeros((N,), dtype=float)
        self.dddC_k_y = numpy.zeros((N,), dtype=float)
        self.dddC_k_q = numpy.zeros((N,), dtype=float)

        # feet matrices

        self.F_kp1_x = numpy.zeros((N,), dtype=float)
        self.F_kp1_y = numpy.zeros((N,), dtype=float)
        self.F_kp1_q = numpy.zeros((N,), dtype=float)

        self.f_k_x = 0.0
        self.f_k_y = 0.0
        self.f_k_q = 0.0

        self.f_x = numpy.zeros((self.nf,), dtype=float)
        self.f_y = numpy.zeros((self.nf,), dtype=float)
        self.f_q = numpy.zeros((self.nf,), dtype=float)

        # zero moment point matrices

        self.Z_kp1_x = numpy.zeros((N,), dtype=float)
        self.Z_kp1_y = numpy.zeros((N,), dtype=float)

        # transformation matrices

        self.Pps = numpy.zeros((N,3), dtype=float)
        self.Ppu = numpy.zeros((N,N), dtype=float)

        self.Pvs = numpy.zeros((N,3), dtype=float)
        self.Pvu = numpy.zeros((N,N), dtype=float)

        self.Pas = numpy.zeros((N,3), dtype=float)
        self.Pau = numpy.zeros((N,N), dtype=float)

        self.Pzs = numpy.zeros((N,3), dtype=float)
        self.Pzu = numpy.zeros((N,N), dtype=float)

        self.v_kp1 = numpy.zeros((N,), dtype=float)
        self.V_kp1 = numpy.zeros((N,nf), dtype=float)

        # initialize transformation matrices

        self._initialize_matrices()


    def _initialize_matrices(self):
        """
        initializes the transformation matrices according to the walking report
        """
        T = self.T
        N = self.N
        h_com = self.h_com
        g = self.g

        for i in range(N):
            # TODO initialize v_k, V_kp1
            # according to state machine from MNaveau

            self.Pzs[i, :] = (1.,   i*T, (i**2*T**2)/2. + h_com/g)
            self.Pps[i, :] = (1.,   i*T,           (i**2*T**2)/2.)
            self.Pvs[i, :] = (0.,    1.,                      i*T)
            self.Pas[i, :] = (0.,    0.,                       1.)

            for j in range(N):
                if j <= i:
                    self.Pzu[i, j] = (3.*(i-j)**2 + 3.*(i-j) + 1.)*T**3/6. + T*h_com/g
                    self.Ppu[i, j] = (3.*(i-j)**2 + 3.*(i-j) + 1.)*T**3/6.
                    self.Pvu[i, j] = (2.*(i-j) + 1.)*T**2/2.
                    self.Pau[i, j] = T

    def simulate(self):
        """
        integrates model for given jerks and feet positions and orientations
        """

        self.  C_kp1_x = self.Pps.dot(self.c_k_x) + self.Ppu.dot(self.dddC_k_x)
        self. dC_kp1_x = self.Pvs.dot(self.c_k_x) + self.Pvu.dot(self.dddC_k_x)
        self.ddC_kp1_x = self.Pas.dot(self.c_k_x) + self.Pau.dot(self.dddC_k_x)

        self.  C_kp1_y = self.Pps.dot(self.c_k_y) + self.Ppu.dot(self.dddC_k_y)
        self. dC_kp1_y = self.Pvs.dot(self.c_k_y) + self.Pvu.dot(self.dddC_k_y)
        self.ddC_kp1_y = self.Pas.dot(self.c_k_y) + self.Pau.dot(self.dddC_k_y)

        self.  C_kp1_q = self.Pps.dot(self.c_k_q) + self.Ppu.dot(self.dddC_k_q)
        self. dC_kp1_q = self.Pvs.dot(self.c_k_q) + self.Pvu.dot(self.dddC_k_q)
        self.ddC_kp1_q = self.Pas.dot(self.c_k_q) + self.Pau.dot(self.dddC_k_q)

        self.Z_kp1_x = self.Pzs.dot(self.c_k_x) + self.Pzu.dot(self.dddC_k_x)
        self.Z_kp1_y = self.Pzs.dot(self.c_k_y) + self.Pzu.dot(self.dddC_k_y)

    def solve(self):
        """
        solves QP for given initial values and jerk constraints
        """
        self._prepare_optimization()


    def _prepare_optimization(self):
        """
        build up QP matrices
        Q = ( Q_x   0 ), p = ( p_x ), x = ( dddC_kp1_x )
            (   0 Q_y )      ( p_y )      (        f_x )
                                          ( dddC_kp1_y )
                                          (        f_y )
        """

        Q_x = self.Q[:self.NXI, self.NXI]
        p_x = self.p[:self.NXI]

        # Q_x = ( x * ), <- x selects the entry
        #       ( * * )
        Q_x[:self.N,:self.N] = \
                0.0

        # Q_x = ( * x ), <- x selects the entry
        #       ( * * )
        Q_x[:self.N,self.N:self.NXI] = \
                -self.weight[2] * self.Pzu.T * self.V_kp1

        # Q_x = ( * * ), <- x selects the entry
        #       ( x * )
        Q_x[self.N:self.NXI,:self.N] = \
                -self.weight[2] * self.Pzu.T * self.V_kp1

        # Q_x = ( * * ), <- x selects the entry
        #       ( * x )
        Q_x[self.N:self.NXI,self.N:self.NXI] = \
                -self.weight[2] * self.V_kp1.T * self.V_kp1
