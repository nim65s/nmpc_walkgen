import os, sys
from walking_generator.base import BaseGenerator
from walking_generator.classic import ClassicGenerator
from walking_generator.interpolation import Interpolation

if __name__ == '__main__':
    gen = ClassicGenerator()
    comx = [0.00124774,0.0,0.0]
    comy = [0.00157175,0.0,0.0]
    comz = 0.08108886
    supportfootx = 0.00949035
    supportfooty = 0.095
    supportfootq = 0.0
    secmarginx = 0.09
    secmarginy = 0.05
    gen._initState(comx,comy,comz,\
            supportfootx,supportfooty,supportfootq,secmarginx,secmarginy)


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
