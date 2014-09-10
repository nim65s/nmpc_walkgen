import os, sys
from walking_generator.base import BaseGenerator
from walking_generator.classic import ClassicGenerator
from walking_generator.interpolation import Interpolation

if __name__ == '__main__':
    gen = ClassicGenerator()
    # define initial state
    comx = [0.06591456,0.07638739,-0.1467377]
    comy = [2.49008564e-02,6.61665254e-02,6.72712187e-01]
    comz = 0.814
    supportfootx = 0.00949035
    supportfooty = 0.095
    supportfootq = 0.0
    secmargin = 0.04
    gen._initState(
        comx,comy,comz,
        supportfootx,supportfooty,supportfootq,
        secmargin,secmargin
    )

    gen.simulate()

    # set reference velocities to zero
    gen.dC_kp1_x_ref[...] = 0.1
    gen.dC_kp1_y_ref[...] = 0.1
    gen.dC_kp1_q_ref[...] = 0.0

    gen.solve()

    for i in range(100):
        print 'iteration: ', i
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
        #raw_input('press return to continue: ')
