import os, sys
import time
import numpy
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from walking_generator.visualization import Plotter
from walking_generator.classic import ClassicGenerator
from walking_generator.interpolation import Interpolation

# instantiate pattern generator
gen = ClassicGenerator(fsm_state='L/R')

# instantiate plotter
show_canvas = True
save_to_file = False
plot = Plotter(gen, show_canvas, save_to_file)

# Pattern Generator Preparation
# set reference velocities to zero
velocity_reference = [0.2,0.0,0.2]
gen.set_velocity_reference(velocity_reference)

gen.set_security_margin(0.09, 0.05)

# set initial values
comx = [0.00949035, 0.0, 0.0]
comy = [0.095,      0.0, 0.0]
comz = 0.814
footx = 0.00949035
footy = 0.095
footq = 0.0
gen.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')
#gen._update_selection_matrices()
sampling_period = 0.1
interpol = Interpolation(0.005,gen)
gen.simulate()
gen._update_data()

# Pattern Generator Event Loop
for i in range(220):
    print 'iteration: ', i
    time = i*0.1

    # change reference velocities
    if 25 <= i < 50:
        velocity_reference = [ 0.2, 0.0, -0.2]
    if 50 <= i < 100:
        velocity_reference = [0.1, 0.2,-0.4]
    if 100 <= i < 150:
        velocity_reference = [ 0.0, 0.2, 0.0]
    if 150 <= i :
        velocity_reference = [ 0.0, 0.0, 0.0]

    # set reference velocities to zero
    gen.set_velocity_reference(velocity_reference)


    gen.dddC_k_q  [...] = 1.0
    gen.dddF_k_qL [...] = 1.0
    gen.dddF_k_qR [...] = 1.0

    # solve QP
    gen.solve()
    gen.simulate()
    interpol.interpolate(time)

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
interpol.save_to_file("./wieber2010python.csv")

