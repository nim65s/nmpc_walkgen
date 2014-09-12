import numpy
from copy import deepcopy
from base import CoMState, ZMPState, BaseTypeFoot
from base import BaseGenerator

class Interpolation(Object):
    """
    Interpolation class of walking pattern generator for humanoids, cf.
    LAAS-UHEI walking report.

    Interpolation Class provides all methods to interpolated from the solution of the
    pattern generator. It interpolate the CoM, the ZMP and the Feet state along the
    whole trajectory with a given interpolation period (input)
    """
    def __init__(self, T=0.005, BG=BaseGenerator() )

        # the generator is supposed to have been initialized before
        self.gen = BG

        self.T = T # QP sampling period
        self.Tc = Tcontrol # sampling period of the robot low level controller
        self.interval = int(self.Tc/self.T)+1 # number of iteration in 100ms
                                              # + the initial state of the next QP iteration

        # initiale states used to interpolate (they should be intialized once ate the beginning of the qp
        # and updated inside the class
        self.curCoM = CoMState()
        self.curCoM.x = self.gen.c_k_q
        self.curCoM.y = self.gen.c_k_q
        self.curCoM.theta = self.gen.c_k_q
        self.curCoM.h_com = self.gen.h_com

        self.curSupport = BaseTypeFoot()
        curSupport.supportFoot = 1
        self.curSwingFoot = BaseTypeFoot()

        self.CoMbuffer = numpy.empty( (self.interval,) , dtype=object ) #buffer conatining the CoM trajectory over 100ms
        self.ZMPbuffer = numpy.empty( (self.interval,) , dtype=object ) #buffer conatining the ZMP trajectory over 100ms
        self.RFbuffer = numpy.empty( (self.interval,) , dtype=object ) #buffer conatining the rigth foot trajectory over 100ms
        self.LFbuffer = numpy.empty( (self.interval,) , dtype=object ) #buffer conatining the left foot trajectory over 100ms
        for i in range(self.interval):
            self.CoMbuffer[i] = deepcopy(self.curCoM)
            self.ZMPbuffer[i] = ZMPState()
            if gen.currentSupport = "left"
                self.RFbuffer[i] = deepcopy(self.curSwingFoot)
                self.LFbuffer[i] = deepcopy(self.curSupport)
            else :
                self.RFbuffer[i] = deepcopy(self.curSupport)
                self.LFbuffer[i] = deepcopy(self.curSwingFoot)

        self.lipm = LIPM(Tcontrol,T,h_com)
        self.fi = FootInterpolation()

    def interpolate(self):

        self.lipm.interpolate(self.curCoM, self.CoMbuffer, self.ZMPbuffer,
                                self.gen.dddC_k_x[0], self.gen.dddC_k_x[1] )

        self.fi.interpolate(time, self.curSupport, self.curSwingFoot,
                            self.gen.f_k_x, self.gen.f_k_y,self.gen.f_k_q,
                            self.LFbuffer, self.RFbuffer)

class LIPM(object):
    """
    LIPM class of walking pattern generator for humanoids, cf.
    LAAS-UHEI walking report.

    LIPM class provides all methods to interpolate from the solution
    of the pattern generator the CoM and the ZMP according
    to the linearized inverted pendulum (Kajita2003, Walking)
    """

    # define some constants
    g = 9.81

    def __init__(self, controlPeriod=0.1, commandPeriod=0.005, h_com=0.81):
        self.Tc=controlPeriod
        self.T=commandPeriod
        self.h_com=h_com
        self.A = numpy.zeros( (3,3) , dtype=float )
        self.B = numpy.zeros( (3,) , dtype=float )
        self.C = numpy.zeros( (3,) , dtype=float )
        self.intervaleSize = self.Tc/self.T+1

        self.initializeSystem()

    def initializeSystem(self):
        T = self.T
        A = self.A
        A[0][0] = 1 ; A[0][1] = T ; A[0][2] = T*T*0.5 ;
        A[1][1] = 1 ; A[1][2] = T ;
        A[2][2] = 1 ;

        B = self.B
        B[0]= T*T*T/6
        B[1]= T*T/2
        B[2]= T

        C = self.C
        C[0]= 1
        C[1]= 0
        C[2]= -self.h_com/self.g

    def interpolate(self, CoMinit, CoMbuffer, ZMPbuffer,\
                    jerkX, jerkY):
        CoMbuffer.resize(self.intervaleSize)
        for i in range(self.intervaleSize):
            CoMbuffer[i].x = A.dot(CoMinit.x) + B.dot(jerkX)
            CoMbuffer[i].y = A.dot(CoMinit.y) + B.dot(jerkY)
            ZMPbuffer[i].x = C.CoMbuffer[i].x
            ZMPbuffer[i].y = C.CoMbuffer[i].y

        CoMinit = deepcopy(CoMbuffer[-1])

class FootInterpolation(object):
    """
    footInterpolation class of walking pattern generator for humanoids, cf.
    LAAS-UHEI walking report.

    footInterpolation class provides all methods to interpolate from the solution
    of the pattern generator. It interpolate the feet trajectory during the QP period
    """

    def __init__(self, QPsamplingPeriod=0.1, NbSamplingPreviewed=16, commandPeriod=0.005,
        FeetDistance=0.2, StepHeight=0.05, stepTime=0.8, doubleSupportTime=0.1):

        self.T = QPsamplingPeriod # QP sampling period
        self.Tc = commandPeriod # Low level control period
        self.feetDist = FeetDistance # Normal Distance between both feet
        self.stepHeigth = StepHeight # Standard maximal step height
        self.polynomeX     = Polynome5() # order 5 polynome for continuity
        self.polynomeY     = Polynome5() #  in position, velocity
        self.polynomeTheta = Polynome5() #  and acceleration
        self.polynomeZ     = Polynome4() # order 5 for a defined middle point
        self.TSS = stepTime - doubleSupportTime # Time of single support
        self.TDS = doubleSupportTime # Time of double support
        self.intervaleSize = int(self.T/self.Tc) # nuber of interpolated sample

    '''
    Update the currentState to be valide at the next iteration
    and fill up the queue containing all the intermediate values
    '''
    def interpolate(self, time, currentSupport,
        CurrentSwingFootPosition,CurrentNonSwingFoot,
        F_k_x, F_k_y, PreviewAngle,
        LeftFootBuffer, RightFootBuffer):

        '''
        |---------------||-------|-----------------------------|-------|
        |     DSP*      ||             single support phase            |
        |               ||take   |      flying                 |landing|
        |               || off   |       phase                 |       |

        * DSP : Double Support Phase
        '''
        # in case of double support the policy is to stay still
        if time+1.5*self.T > currentSupport.timeLimit :
            if currentSupport.foot == "left" :
                left = CurrentNonSwingFoot
                right = CurrentSwingFootPosition
            else :
                left = CurrentSwingFootPosition
                right = CurrentNonSwingFoot
            for i in range(self.intervaleSize):
                LeftFootBuffer[i] = deepcopy(left)
                RightFootBuffer[i] = deepcopy(right)
            # we define the z trajectory in the double support phase
            # to allow the robot to take off and land
            # during the whole singletime
            self.polynomeZ.setParameters(self.TSS,self.stepHeigth,
                        CurrentSwingFootPosition.z,CurrentSwingFootPosition.dz)

        elif time+1.5*self.T < currentSupport.timeLimit :

            # Deal with the lift off time and the landing time. During those period
            # the foot do not move along the x and y axis.

            # this is counted from the last double support phase
            localInterpolationStartTime = time - (currentSupport.timeLimit -(self.TSS+self.TDS) )
            # this coeffincient indicates how long you allow the foot
            # to take off AND land (in %)
            moduleSupportCoefficient = 0.9
            # this time indicates how long the foot will move in x, y and theta
            UnlockedSwingPeriod = self.TSS * moduleSupportCoefficient
            # this time indicates the time limit where the foot should have reach
            # lift off enough to move in x, y and theta
            endOfLiftoff = 0.5 * (self.TSS-UnlockedSwingPeriod)
            # This time show the time where the foot has flight in the air
            SwingTimePassed = 0.0
            if localInterpolationStartTime > endOfLiftoff:
                SwingTimePassed = localInterpolationStartTime - endOfLiftoff
            # This time is the time remaining before the landing
            timeInterval = UnlockedSwingPeriod - SwingTimePassed
            # this time indicates the time limit where the foot should have reach its goal
            # and needs to land
            startLanding = endOfLiftoff + UnlockedSwingPeriod

            # Set the polynomes
            csf = CurrentSwingFootPosition

            self.polynomeX.setParameters(timeInterval,F_k_x,csf.x,csf.dx,csf.ddx)
            self.polynomeY.setParameters(timeInterval,F_k_y,csf.y,csf.dy,csf.ddy)
            self.polynomeTheta.setParameters(timeInterval,PreviewAngle,\
                                                csf.theta,csf.dtheta,csf.ddtheta)

            # begin the interpolation
            LeftFootBuffer = numpy.resize(LeftFootBuffer, self.intervaleSize)
            RightFootBuffer = numpy.resize(RightFootBuffer, self.intervaleSize)

            for i in range(self.intervaleSize):
                if currentSupport.foot == "left" :
                    # sf = swing foot ; nsf = non swinging foot
                    sf = RightFootBuffer
                    nsf = LeftFootBuffer
                else :
                    sf = LeftFootBuffer
                    nsf = RightFootBuffer

                # the non swing foot stay still
                nsf[i].supportFoot = 1

                Ti = self.Tc * i # interpolation time
                Tlocal = localInterpolationStartTime + Ti

                # if we are landing or lifting the foot, do not modify the x,y and theta
                if localInterpolationStartTime < endOfLiftoff:
                    Tr = Ti - endOfLiftoff # Tr = remaining time
                    self.computeXYTheta(sf[i],Tr)
                else:
                    self.computeXYTheta(sf[i],Ti)

                sf[i].z = self.polynomeZ.compute(Tlocal)
                sf[i].dz = self.polynomeZ.computeDerivative(Tlocal)
                sf[i].ddz = self.polynomeZ.computeSecDerivative(Tlocal)

            if localInterpolationStartTime < endOfLiftoff:
                # Compute the next iteration state
                self.computeXYTheta(csf,self.Tc*self.intervaleSize - endOfLiftoff)
            else:
                self.computeXYTheta(csf,self.Tc*self.intervaleSize)


    def computeXYTheta(self,foot,t):
        # compute the foot states at time t
        foot.  x = self.polynomeX.compute(t)
        foot. dx = self.polynomeX.computeDerivative(t)
        foot.ddx = self.polynomeX.computeSecDerivative(t)

        foot.  y = self.polynomeY.compute(t)
        foot. dy = self.polynomeY.computeDerivative(t)
        foot.ddy = self.polynomeY.computeSecDerivative(t)

        foot.  theta = self.polynomeTheta.compute(t)
        foot. dtheta = self.polynomeTheta.computeDerivative(t)
        foot.ddtheta = self.polynomeTheta.computeSecDerivative(t)

class Polynome(object):
    """
    Polynome class of walking pattern generator for humanoids, cf.
    LAAS-UHEI walking report.

    Polynome class provides basic mathematics tools for the interpolation
    """
    def __init__(self,degree):
        self.degree = degree
        self.coef = numpy.zeros( (degree+1,) , dtype=float )

    def compute(self,time):
        if time > self.FT :
            time = self.FT
        if time < 0 :
            time = 0
        r = 0.0; t = 1.0
        for i in range(self.coef.shape[0]):
            r = r + self.coef[i] * t
            t = t * time ;
        return r

    def computeDerivative(self,time):
        if time > self.FT :
            time = self.FT
        if time < 0 :
            time = 0
        r = 0.0; t = 1.0
        for i in range(self.coef.shape[0])[1:]:
            r = r + i* self.coef[i] * t
            t = t * time ;
        return r

    def computeSecDerivative(self,time):
        if time > self.FT :
            time = self.FT
        if time < 0 :
            time = 0
        r = 0.0; t = 1.0
        for i in range(self.coef.shape[0])[2:]:
            r = r + i * (i-1) * self.coef[i] * t
            t = t * time ;
        return r

class Polynome5(Polynome):
    """
    Polynome5 class of walking pattern generator for humanoids, cf.
    LAAS-UHEI walking report.

    Polynome5 class provides all methods to interpolate from a point A
    to a point A with certain velocity an acceleration with a 5th degree
    polynome
    """
    def __init__(self):
        self.FT = 0.0
        self.FP = 0.0
        self.IP = 0.0
        self.IS = 0.0
        self.IA = 0.0
        Polynome.__init__(self,5)
        self.setParameters(self.FT,self.FP,self.IP,self.IS,self.IA)

    def setParameters(self,FinalTime,FinalPosition,InitialPosition,InitialSpeed,InitialAcceleration):
        self.FT = deepcopy(FinalTime)
        self.FP = deepcopy(FinalPosition)
        self.IP = deepcopy(InitialPosition)
        self.IS = deepcopy(InitialSpeed)
        self.IA = deepcopy(InitialAcceleration)

        FT = self.FT
        FP = self.FP
        IP = self.IP
        IS = self.IS
        IA = self.IA

        self.coef = numpy.zeros( (6,) , dtype=float )

        self.coef[0] = IP
        self.coef[1] = IS
        self.coef[2] = IA*0.5

        T = FT*FT*FT
        if T == 0 :
            self.coef[3] = self.coef[4] = self.coef[5] = 0.0
        else :
            self.coef[3] = (-1.5*IA*FT*FT - 6.0*IS*FT - 10.0*IP + 10.0*FP)/T
            T = T * FT
            self.coef[4] = ( 1.5*IA*FT*FT + 8.0*IS*FT + 15.0*IP - 15.0*FP)/T
            T = T * FT
            self.coef[5] = (-0.5*IA*FT*FT - 3.0*IS*FT -  6.0*IP +  6.0*FP)/T

class Polynome4(Polynome):
    """
    Polynome4 class of walking pattern generator for humanoids, cf.
    LAAS-UHEI walking report.

    Polynome4 class provides all methods to interpolate from a point A
    to a point A with certain velocity an acceleration with a 4th degree
    polynome
    """
    def __init__(self):
        self.FT = 0.0
        self.MP = 0.0
        Polynome.__init__(self,4)
        self.setParameters(self.FT,self.MP,0.0,0.0)

    def setParameters(self,FinalTime,MiddlePosition,InitialPosition,InitialSpeed):
        self.FT = deepcopy(FinalTime)
        self.MP = deepcopy(MiddlePosition)
        FT = self.FT
        MP = self.MP
        IP = InitialPosition
        IS = InitialSpeed
        self.coef[0] = deepcopy(IP)
        self.coef[1] = deepcopy(IS)

        T = FT*FT
        if T == 0 :
            self.coef[2] = self.coef[3] = self.coef[4] = 0.0
        else :
            self.coef[2] = (-4.0*IS*FT - 11.0*IP + 16.0*MP)/T
            T = T * FT
            self.coef[3] = ( 5.0*IS*FT + 18.0*IP - 32.0*MP)/T
            T = T * FT
            self.coef[4] = (-2.0*IS*FT -  8.0*IP + 16.0*MP)/T
