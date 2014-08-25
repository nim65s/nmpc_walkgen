import numpy

class BaseGenerator(object):
    """
    Base class of walking pattern generator for humanoids, cf.
    LAAS-UHEI walking report.

    BaseClass provides all matrices and timestepping methods that are
    defined for the pattern generator family. In derived classes different
    problems and their solvers can be realized.
    """
    # define some constants
    g = 9.81

    def __init__(self, N=15, T=0.1, nf=2, h_com=0.81, time_step=0.8):
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

        time_step : float
            Time for the robot to make 1 step
        """
        self.N = N
        self.T = T
        self.nf = nf
        self.h_com = h_com
        self.time_step = time_step

        # objective weights

        self.a = 1.0 # weight for CoM velocity tracking
        self.b = 0.0 # weight for CoM average velocity tracking
        self.c = 1.0 # weight for ZMP reference tracking
        self.d = 1.0 # weight for jerk minimization

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

        self.F_k_x = numpy.zeros((self.nf,), dtype=float)
        self.F_k_y = numpy.zeros((self.nf,), dtype=float)
        self.F_k_q = numpy.zeros((self.nf,), dtype=float)

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

        """
        NOTE number of foot steps in prediction horizon changes between
        nf and nf+1, because when robot takes first step nf steps are
        planned on the prediction horizon, which makes a total of nf+1 steps.
        """
        self.v_kp1 = numpy.zeros((N,), dtype=float)
        self.V_kp1 = numpy.zeros((N,nf), dtype=float)

        # initialize transformation matrices

        self._initialize_matrices()


    def _initialize_matrices(self):
        """
        initializes the transformation matrices according to the walking report
        """
        time_step = self.time_step
        T = self.T
        N = self.N
        nf = self.nf
        h_com = self.h_com
        g = self.g
        
        """
        # TODO initialize v_k, V_kp1
        # according to state machine from MNaveau
        """

        for i in range(N):
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

        for i in range((int)(time_step/T)):
            self.v_kp1[i] = 1
        for i in range(N-(int)(time_step/T)):
            self.v_kp1[i+time_step/T] = 0
                
        step = 0
        for i in range (N) :
            step = (int)( i / (time_step/T) )
            print "step = "
            print step
            print "i = "
            print i
            print "j = "
            print j
            for j in range (nf):
                self.V_kp1[i,j] = (int)(i+1>(int)(time_step/T) and j==step-1)
        
        print self.V_kp1
        print self.v_kp1
 
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

    def update(self):
        """
        update v_kp1 and V_kp1
        """
        nf = self.nf
        nstep = (int)(self.time_step/self.T)
        N = self.N

        numpy.delete(self.v_kp1,0,axis=None)
        numpy.append(self.v_kp1,[0],axis=0)
        if self.v_kp1[0]==0:
            nf = 1
            for i in range(nstep):
                self.v_kp1[i] = 1
            for i in range(N-nstep):
                self.v_kp1[i+time_step/T] = 0
            step = 0
            for i in range (N) :
                step = (int)( i / (time_step/T) )
                for j in range (nf):
                    self.V_kp1[i,j] = (int)(i+1>(int)(time_step/T) and j==step-1)
        
        else:
            numpy.delete(self.V_kp1,0,axis=None)
            numpy.delete(self.V_kp1,0,axis=None)
        print self.V_kp1
        print self.v_kp1

    def solve(self):
        """
        Solve problem on given prediction horizon with implemented solver.
        """
        err_str = 'Please derive from this class to implement your problem and solver'
        raise NotImplementedError(err_str)
