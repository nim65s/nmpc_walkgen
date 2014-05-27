import numpy

class Generator(object):
    """
    Prototype of Walking Pattern Generator, cf.

    TODO cite paper
    *
    *
    *
    """
    def __ini__(self, N=16, dt=0.1):
        self.N = N
        self.dt = dt
        self.h_com = 10.0
        self.g = 9.81

        # center of mass matrices

        self.   C_kp1_x = numpy.zeros((N,), dtype=float)
        self.  dC_kp1_x = numpy.zeros((N,), dtype=float)
        self. ddC_kp1_x = numpy.zeros((N,), dtype=float)
        self.dddC_kp1_x = numpy.zeros((N,), dtype=float)

        self.   C_kp1_y = numpy.zeros((N,), dtype=float)
        self.  dC_kp1_y = numpy.zeros((N,), dtype=float)
        self. ddC_kp1_y = numpy.zeros((N,), dtype=float)
        self.dddC_kp1_y = numpy.zeros((N,), dtype=float)

        self.   C_kp1_q = numpy.zeros((N,), dtype=float)
        self.  dC_kp1_q = numpy.zeros((N,), dtype=float)
        self. ddC_kp1_q = numpy.zeros((N,), dtype=float)
        self.dddC_kp1_q = numpy.zeros((N,), dtype=float)

        # feet matrices

        self.F_kp1_x = numpy.zeros((N,), dtype=float)
        self.F_kp1_y = numpy.zeros((N,), dtype=float)
        self.F_kp1_q = numpy.zeros((N,), dtype=float)

        self.V_kp1 = numpy.zeros((N,N), dtype=float)

        # transformation matrices

        self.Pps = numpy.zeros((N,3), dtype=float)
        self.Ppu = numpy.zeros((N,N), dtype=float)

        self.Pvs = numpy.zeros((N,3), dtype=float)
        self.Pvu = numpy.zeros((N,N), dtype=float)

        self.Pas = numpy.zeros((N,3), dtype=float)
        self.Pau = numpy.zeros((N,N), dtype=float)

        self.Pzs = numpy.zeros((N,3), dtype=float)
        self.Pzu = numpy.zeros((N,N), dtype=float)

        self._initialize_transformation_matrices()

    def _initialize_transformation_matrices(self):
        """
        initializes the transformation matrices according to the walking report
        """
        T = self.dt
        N = self.N
        h_com = self.h_com
        g = self.g

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

