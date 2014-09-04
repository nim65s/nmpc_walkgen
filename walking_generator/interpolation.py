import numpy
from base import CoMState, ZMPState, BaseTypeFoot

class Interpolation(object):
    """
    Interpolation class of walking pattern generator for humanoids, cf.
    LAAS-UHEI walking report.

    Interpolation Class provides all methods to interpolated from the solution of the
    pattern generator. It interpolate the CoM, the ZMP and the Feet state along the
    whole trajectory with a given interpolation period (input)
    """
    def __init__(self, T=0.005, Tcontrol=0.1, Tstep=0.8, h_com=0.81,
        initCoM=CoMState(), initLeftFoot=BaseTypeFoot(),
        initRightFoot=BaseTypeFoot()):
        self.T = T
        self.Tc = Tcontrol
        self.interval = int(self.Tc/self.T)

        self.CoMbuffer = numpy.empty( (self.interval,) , dtype=object )
        self.ZMPbuffer = numpy.empty( (self.interval,) , dtype=object )
        for i in range(self.interval):
            self.CoMbuffer[i] = CoMState()
            self.ZMPbuffer[i] = ZMPState()

        self.curCoM = initCoM
        if initLeftFoot.supportFoot == 1:
            self.curSupport = initLeftFoot
            self.curSwingFoot = initRightFoot
        else:
            self.curSupport = initRightFoot
            self.curSwingFoot = initLeftFoot
        self.lipm = LIPM(Tcontrol,T,h_com)
        self.fi = FootInterpolation()

    def interpolate(self, F_k_x, F_k_y, jerkX, jerkY,\
                    CoMbuffer, ZMPbuffer, LeftFootBuffer, RightFootBuffer):

        self.lipm.interpolate(self, self.curCoM, CoMbuffer, ZMPbuffer,jerkX, jerkY)
        self.curCoM = CoMbuffer[-1:]

        self.fi.interpolate(time, currentSupport, currentSwingFootPosition,\
                            F_k_x, F_k_y, PreviewAngle,LeftFootBuffer, RightFootBuffer)
        if LeftFootBuffer[-1:].supportFoot == 1:
            self.curSupport = LeftFootBuffer[-1:]
            self.curSwingFoot = RightFootBuffer[-1:]
        else:
            self.curSupport = RightFootBuffer[-1:]
            self.curSwingFoot = LeftFootBuffer[-1:]


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
        self.intervaleSize = self.Tc/self.T

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
        for i in range(self.intervaleSize+1):
            CoMbuffer[i].x = A.dot(CoMinit.x) + B.dot(jerkX)
            CoMbuffer[i].y = A.dot(CoMinit.y) + B.dot(jerkY)
            ZMPbuffer[i].x = C.CoMbuffer[i].x
            ZMPbuffer[i].y = C.CoMbuffer[i].y

class FootInterpolation(object):
    """
    footInterpolation class of walking pattern generator for humanoids, cf.
    LAAS-UHEI walking report.

    footInterpolation class provides all methods to interpolate from the solution
    of the pattern generator. It interpolate the feet trajectory during the QP period
    """

    def __init__(self, QPsamplingPeriod=0.1, NbSamplingPreviewed=16, controlPeriod=0.005,
        FeetDistance=0.2, StepHeight=0.05, stepTime=0.8, doubleSupportTime=0.1):
        self.T = QPsamplingPeriod
        self.Tc = controlPeriod
        self.N = NbSamplingPreviewed
        self.feetDist = FeetDistance
        self.stepHeigth = StepHeight
        self.polynomeX     = Polynome5()
        self.polynomeY     = Polynome5()
        self.polynomeTheta = Polynome5()
        self.ploynomeZ     = Polynome4()
        self.TSS = stepTime - doubleSupportTime
        self.TDS = doubleSupportTime

    def interpolate(self, time, currentSupport,
        currentSwingFootPosition,
        F_k_x, F_k_y, PreviewAngle,
        LeftFootBuffer, RightFootBuffer):

        # Deal with the lift off time and the landing time. During those period
        # the foot do not move along the x and y axis.
        localInterpolationStartTime = Time - (currentSupport.timeLimit - self.TSS-self.TDS)
        moduleSupportCoefficient = 0.9
        UnlockedSwingPeriod = self.TSS * moduleSupportCoefficient
        endOfLiftoff = 0.5(TSS-UnlockedSwingPeriod)
        startLanding = endOfLiftoff + UnlockedSwingPeriod
        SwingTimePassed = 0.0
        if localInterpolationStartTime > endOfLiftoff:
            SwingTimePassed = localInterpolationStartTime - endOfLiftoff
        timeInterval = UnlockedSwingPeriod - endOfLiftoff

        # Set the polynomes
        csf = currentSwingFootPosition
        self.polynomeX.setParameters(timeInterval,F_k_x,csf.x,csf.dx,csf.ddx)
        self.polynomeY.setParameters(timeInterval,F_k_y,csf.y,csf.dy,csf.ddy)
        self.polynomeTheta.setParameters(timeInterval,PreviewAngle,\
                                            csf.theta,csf.dtheta,csf.ddtheta)
        if currentSupport.ds == 1 :
            self.polynomeZ.setParameters(self.TSS,self.stepHeigth,csfp.z,csfp.dz)

        # begin the interpolation
        LeftFootBuffer.resize(int(self.T/self.Tc))
        RightFootBuffer.resize(int(self.T/self.Tc))
        for i in range(int(self.T/self.Tc)):
            if currentSupport.foot == "left" :
                # sf = swing foot ; nsf = non swinging foot
                sf = RightFootBuffer[i]
                nsf = LeftFootBuffer[i]
            else :
                sf = LeftFootBuffer[i]
                nsf = RightFootBuffer[i]

            # the non swing foot stay still
            nsf = currentSupport
            nsf.supportFoot = 1

            Ti = Tc * i # interpolation time
            # if we are landing or lifting the foot, do not modify the x,y and theta
            if Ti <= endOfLiftoff and Ti >= startLanding :
                sf.x = csf.x
                sf.y = csf.y
                sf.theta = csf.theta
            elif localInterpolationStartTime < endOfLiftoff:
                Tr = Ti - endOfLiftoff # Tr = remaining time
                self.computeXYTheta(sf,Tr)
            else:
                self.computeXYTheta(sf,Ti)


    def computeXYTheta(self,sf,t):
        # compute the foot states at time t
        sf.  x = self.polynomeX.compute(t)
        sf. dx = self.polynomeX.compute(t)
        sf.ddx = self.polynomeX.compute(t)

        sf.  y = self.polynomeY.compute(t)
        sf. dy = self.polynomeY.compute(t)
        sf.ddy = self.polynomeY.compute(t)

        sf.  theta = self.polynomeTheta.compute(t)
        sf. dtheta = self.polynomeTheta.compute(t)
        sf.ddtheta = self.polynomeTheta.compute(t)

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
            time = FT
        r = 0.0; t = 1.0
        for i in range(self.coef.shape[0]):
            r += i * self.coef[i] * t
            t *= time ;
        return r

    def computeDerivative(self,time):
        if time > self.FT :
            time = FT
        r = 0.0; t = 1.0
        for i in range(self.coef.shape[0])[1:]:
            r += i*(i-1) * self.coef[i] * t
            t *= time ;
        return r

    def computeSecDerivative(self,time):
        if time > self.FT :
            time = FT
        r = 0.0; t = 1.0
        for i in range(self.coef.shape[0])[2:]:
            r += i*(i-1)*(i-2) * self.coef[i] * t
            t *= time ;
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
        self.FT = FT = 0.0
        self.FP = FP = 0.0
        self.IP = IP = 0.0
        self.IS = IS = 0.0
        self.IA = IA = 0.0
        initialAcceleration = 0.0
        Polynome.__init__(self,5)
        self.setParameters(FT,FP,IP,IS,IA)

    def setParameters(self,FinalTime,FinalPosition,InitialPosition,InitialSpeed,InitialAcceleration):
        self.FT = FT = FinalTime
        self.FP = FP = FinalPosition
        self.IP = IP = InitialPosition
        self.IS = IS = InitialSpeed
        self.IA = IA = InitialAcceleration
        initialAcceleration = 0.0
        self.coef = numpy.zeros( (6,) , dtype=float )

        self.coef[0] = IP
        self.coef[1] = IS
        self.coef[2] = IA*0.5
        T = self.FT*self.FT*self.FT
        if T == 0 :
            self.coef[3] = self.coef[4] = self.coef[5] = 0.0
        else :
            self.coef[3] = (-1.5*IA*FT*FT - 6.0*IS*FT - 10.0*IP + 10.0*FP)/T
            T = T * self.FT
            self.coef[4] = ( 1.5*IA*FT*FT + 8.0*IS*FT + 15.0*IP - 15.0*FP)/T
            T = T * self.FT
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
        self.FT = FT = 0.0
        self.MP = MP = 0.0
        Polynome.__init__(self,4)
        self.setParameters(FT,MP,0.0,0.0)

    def setParameters(self,FinalTime,MiddlePosition,InitialPosition,InitialSpeed):
        self.FT = FT = FinalTime
        self.MP = MP = MiddlePosition
        IP = InitialPosition
        IS = InitialSpeed
        self.coef[0] = IP
        self.coef[1] = IS

        T = FT*FT
        if T == 0 :
            self.coef[2] = self.coef[3] = self.coef[4] = 0.0
        else :
            self.coef[2] = (-4.0*IS*FT - 11.0*IP + 16.0*MP)/T
            T = T * FT
            self.coef[3] = ( 5.0*IS*FT + 18.0*IP - 32.0*MP)/T
            T = T * FT
            self.coef[4] = (-2.0*IS*FT -  8.0*IP + 16.0*MP)/T
