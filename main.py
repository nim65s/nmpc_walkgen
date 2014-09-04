import os, sys
from walking_generator.base import BaseGenerator
from walking_generator.classic import ClassicGenerator
from walking_generator.interpolation import Interpolation

if __name__ == '__main__':
    gen = ClassicGenerator()
    gen.simulate()
    gen.solve()

    # set reference velocities to zero
    gen.dC_kp1_x_ref[...] = 0.1
    gen.dC_kp1_y_ref[...] = 0.1
    gen.dC_kp1_q_ref[...] = 0.0

    for i in range(1):
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
        print 'gen.ori_H:\n', gen.ori_H
        print 'gen.pos_H:\n', gen.pos_H
        gen.solve()
        raw_input('press return to continue: ')
