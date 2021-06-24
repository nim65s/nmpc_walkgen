import os, sys
import time
import numpy
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from walking_generator.visualization import Plotter
from walking_generator.combinedqp import NMPCGenerator
# from walking_generator.combinedqp_init import NMPCGeneratorInit
from walking_generator.interpolation import Interpolation

# instantiate pattern generator
nmpc    = NMPCGenerator(fsm_state='L/R')

# Pattern Generator Preparation
nmpc.   set_security_margin(0.09, 0.05)

# instantiate plotter
show_canvas  = True
save_to_file = False
nmpc_p    = Plotter(nmpc,    show_canvas, save_to_file)

# set initial values
comx = [0.00679821, 0.0, 0.0]#[0.00679821, 0.0, 0.0]
comy = [0.08693283,0.0, 0.0] #[0.08693283,0.0, 0.0] #0.03
comz = 8.786810585901939641e-01 
footx = 0.00949035#-0.008
footy = 0.095
footq = 0.0
# Fx = [0.00949035,0.00949035]
# Fy = [0.095,-0.095]
# Fq = [0.0,0.0]
nmpc.   set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

interpolNmpc = Interpolation(0.001,nmpc)
# initial reference velocity
velocity_reference = [0.2, 0.0,0.0]

nb_step = 2

# Pattern Generator Event Loop
for i in range(16*nb_step):
    print 'iteration: ', i
    time = i*0.1

    # if i == 16:
    #     nmpc    = NMPCGenerator(fsm_state='D')
    #     nmpc.   set_security_margin(0.09, 0.05)
    #     nmpc.   set_initial_values(comx, comy, comz, footx, footy, footq,\
    #     foot, comq, Fx, Fy, Fq) # <== ICI Ajouter ces F !!! ==>
    # # change reference velocities
    # if 16 <= i < 16*(nb_step-3):
    #     velocity_reference = [ 0.2, 0.0, 0.0]
    # if 50 <= i < 150:
    #     velocity_reference = [0.1, 0.2,-0.4]
    # if 150 <= i < 200:
    #     velocity_reference = [ 0.0, 0.2, 0.0]
    # if 200 <= i :
    #     velocity_reference = [ 0.0, 0.0, 0.0]

    # if 16*(nb_step-3) <= i:
    #     velocity_reference = [ 0.0, 0.0, 0.0]

    # set reference velocities to zero
    nmpc.   set_velocity_reference(velocity_reference)

    # solve QP
    nmpc.   solve()
    nmpc.   simulate()
    interpolNmpc.interpolate(time)

    # initial value embedding by internal states and simulation
    comx, comy, comz, footx, footy, footq, foot, comq = \
    nmpc.update()
    # print(footy)
    nmpc.set_initial_values(comx, comy, comz, footx, footy, footq,\
    foot, comq)
    if show_canvas:
        nmpc_p.update()

nmpc.   data.save_to_file('./nmpc_alone.json')

show_canvas  = False
save_to_file = True

nmpc_p    = Plotter(
    generator=None, show_canvas=show_canvas, save_to_file=save_to_file,
    filename='./nmpc_alone',    fmt='pdf'
)

nmpc_p   .load_from_file('./nmpc_alone.json')
nmpc_p   .update()
nmpc_p   .create_data_plot()

interpolNmpc.save_to_file("./nmpc_alone.csv")
