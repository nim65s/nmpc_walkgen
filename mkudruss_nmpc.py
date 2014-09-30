import os, sys
import time
import numpy
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from walking_generator.visualization import Plotter
from walking_generator.classic import ClassicGenerator
from walking_generator.combinedqp import NMPCGenerator

# instantiate pattern generator
gen = NMPCGenerator(fsm_state='L/R')

# instantiate plotter
show_canvas = True
save_to_file = False
plot = Plotter(gen, show_canvas, save_to_file)

# Pattern Generator Preparation
# set reference velocities to zero
gen.set_velocity_reference([0.2,0.0,-0.0])

gen.set_security_margin(0.04, 0.04)

# set initial values
comx = [0.00949035, 0.0, 0.0]
comy = [0.095,      0.0, 0.0]
comz = 0.814
footx = 0.00949035
footy = 0.095
footq = 0.0
gen.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

gen.simulate()
gen._update_data()

# Pattern Generator Event Loop
for i in range(160):
    print 'iteration: ', i

    if 50 <= i < 100:
        gen.set_velocity_reference([0.2,0.0,-0.0])
    if 100 <= i < 130:
        gen.set_velocity_reference([0.2,0.0,-0.0])
    if 130 <= i:
        gen.set_velocity_reference([0.2,0.0,-0.0])

    gen.dddC_k_q  [...] = 1.0
    gen.dddF_k_qL [...] = 1.0
    gen.dddF_k_qR [...] = 1.0

    # solve QP
    gen.solve()
    gen.simulate()

    #print 'gen.E_F\n', gen.E_F
    #print 'gen.E_F_bar\n', gen.E_F_bar
    #print 'gen.A_rot_eq\n',   gen.A_rot_eq
    #print 'gen.B_rot_eq\n',   gen.B_rot_eq
    #print 'gen.A_rot_ineq\n', gen.A_rot_ineq
    #print 'gen.B_rot_ineq\n', gen.B_rot_ineq

    #print 'gen._ori_Q:\n',    gen._ori_Q
    #print 'gen._ori_p:\n',    gen._ori_p
    #print 'gen.F_kp1_q:\n',   gen.F_kp1_q
    #print 'gen.F_k_q:\n',     gen.F_k_q
    #print 'gen.F_kp1_qL:\n',  gen.F_kp1_qL
    #print 'gen.F_kp1_qR:\n',  gen.F_kp1_qR
    #print 'gen.dddF_k_qL:\n', gen.dddF_k_qL
    #print 'gen.dddF_k_qR:\n', gen.dddF_k_qR

    # initial value embedding by internal states and simulation
    comx, comy, comz, footx, footy, footq, foot, comq= \
    gen.update()
    gen.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)
    #plot.update()

    #raw_input('press key:')
    #time.sleep(0.1)

gen.data.save_to_file('./data.json')

