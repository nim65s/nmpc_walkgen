import os, sys
from walking_generator.base import *
from walking_generator.classic import ClassicGenerator
from walking_generator.interpolation import FootInterpolation


if __name__ == '__main__':
#    gen = ClassicGenerator()
#    comx = [0.00124774,0.0,0.0]
#    comy = [0.00157175,0.0,0.0]
#    comz = 0.814
#    supportfootx = 0.00949035
#    supportfooty = 0.095
#    supportfootq = 0.0
#    secmarginx = 0.09
#    secmarginy = 0.05
#    gen._initState(comx,comy,comz,\
#            supportfootx,supportfooty,supportfootq,secmarginx,secmarginy)
#    gen.simulate()

#    # set reference velocities to zero
#    gen.dC_kp1_x_ref[...] = 0.2
#    gen.dC_kp1_y_ref[...] = 0.0
#    gen.dC_kp1_q_ref[...] = 0.0

#    samplingPeriod = 0.005
#    inter = Interpolation(samplingPeriod, gen.T, gen.T_step,gen.h_com)

#    comTraj = numpy.empty( (,) , dtype=object )
#    zmpTraj = numpy.empty( (,) , dtype=object )
#    leftFootTraj = numpy.empty( (,) , dtype=object )
#    rightFootTraj = numpy.empty( (,) , dtype=object )

#    gen.solve()

#    for i in range(100):
#        print 'iteration: ', i
#        gen.c_k_x[0] = gen.  C_kp1_x[0]
#        gen.c_k_x[1] = gen. dC_kp1_x[0]
#        gen.c_k_x[2] = gen.ddC_kp1_x[0]

#        gen.c_k_y[0] = gen.  C_kp1_y[0]
#        gen.c_k_y[1] = gen. dC_kp1_y[0]
#        gen.c_k_y[2] = gen.ddC_kp1_y[0]

#        gen.c_k_q[0] = gen.  C_kp1_q[0]
#        gen.c_k_q[1] = gen. dC_kp1_q[0]
#        gen.c_k_q[2] = gen.ddC_kp1_q[0]

#        gen.update()
#        gen.solve()

#        inter.interpolateCoMZMP(gen.F_k_x,gen.F_k_y,jerkX, jerkY,comTraj, zmpTraj)
#        #raw_input('press return to continue: ')

    currentSupport = BaseTypeSupportFoot()
    currentSupport.x = 0.0
    currentSupport.y = 0.0
    currentSupport.theta = 0.0
    currentSupport.foot = "left"
    currentSupport.ds = 0
    currentSupport.stepNumber = 0
    currentSupport.timeLimit = 0.9

    supportFootDeque = numpy.empty( (9,) , dtype=object )
    for i in range (supportFootDeque.shape[0]):
        supportFootDeque[i] = deepcopy(currentSupport)

    supportFootDeque[0].ds=1
    supportFootDeque[0].timeLimit = 0.1
    supportFootDeque[8].ds=1

    FI = FootInterpolation()
    time = 0.0

    currentSwingFootPosition = BaseTypeFoot()
    currentSwingFootPosition.x = 0.0
    currentSwingFootPosition.y = 0.0
    currentSwingFootPosition.theta = 0.0
    currentSwingFootPosition.dx = 0
    currentSwingFootPosition.dy = 0
    currentSwingFootPosition.dtheta = 0
    currentSwingFootPosition.ddx = 0
    currentSwingFootPosition.ddy = 0
    currentSwingFootPosition.ddtheta = 0
    currentSwingFootPosition.supportFoot = 0

    currentNonSwingFootPosition = BaseTypeFoot()
    currentNonSwingFootPosition.x = 0.0
    currentNonSwingFootPosition.y = 0.0
    currentNonSwingFootPosition.theta = 0.0
    currentNonSwingFootPosition.dx = 0
    currentNonSwingFootPosition.dy = 0
    currentNonSwingFootPosition.dtheta = 0
    currentNonSwingFootPosition.ddx = 0
    currentNonSwingFootPosition.ddy = 0
    currentNonSwingFootPosition.ddtheta = 0
    currentNonSwingFootPosition.supportFoot = 1

    PreviewAngle = 0.0
    F_k_x = 0.2
    F_k_y = 0.2

    LeftFootBuffer = numpy.empty( (20,) , dtype=object )
    for i in range(LeftFootBuffer.shape[0]):
        LeftFootBuffer[i] = BaseTypeFoot()

    RightFootBuffer = numpy.empty( (20,) , dtype=object )
    for i in range(LeftFootBuffer.shape[0]):
        RightFootBuffer[i] = BaseTypeFoot()

    LeftFootTraj = numpy.empty( (0,) , dtype=object )
    RightFootTraj = numpy.empty( (0,) , dtype=object )
    LFx = numpy.empty( (0,) , dtype=object )
    RFx = numpy.empty( (0,) , dtype=object )

    for i in range(supportFootDeque.shape[0]):
        time = i * 0.1
        FI.interpolate(time, supportFootDeque[i],
        currentSwingFootPosition, currentNonSwingFootPosition ,
        F_k_x, F_k_y, PreviewAngle,
        LeftFootBuffer, RightFootBuffer)

        LeftFootTrajX = numpy.zeros( LeftFootBuffer.shape , dtype=float)
        RightFootTrajX = numpy.zeros( RightFootBuffer.shape , dtype=float)
        for j in range(RightFootBuffer.shape[0]):
            LeftFootTrajX[j] = LeftFootBuffer[j].z
            RightFootTrajX[j] = RightFootBuffer[j].z

        LeftFootTrajX.tofile("LeftFootBuffer"+format(i),sep="\n")
        RightFootTrajX.tofile("RightFootBuffer"+format(i),sep="\n")

        LFx = numpy.append(LFx,LeftFootTrajX)
        RFx = numpy.append(RFx,RightFootTrajX)

    LFx.tofile("lfttraj.txt",sep="\n")
    RFx.tofile("rfttraj.txt",sep="\n")

    print "done"



















