import os, sys
import time
import numpy
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from walking_generator.visualization import Plotter
from walking_generator.classic import ClassicGenerator
from walking_generator.combinedqp import NMPCGenerator

# instantiate pattern generator
nmpc    = NMPCGenerator(fsm_state='L/R')
classic = ClassicGenerator(fsm_state='L/R')

# Pattern Generator Preparation
nmpc.   set_security_margin(0.04, 0.04)
classic.set_security_margin(0.04, 0.04)

# instantiate plotter
show_canvas = True
save_to_file = False
nmpc_p    = Plotter(nmpc,    show_canvas, save_to_file)
classic_p = Plotter(classic, show_canvas, save_to_file)

# set initial values
comx = [0.00949035, 0.0, 0.0]
comy = [0.095,      0.0, 0.0]
comz = 0.814
footx = 0.00949035
footy = 0.095
footq = 0.0

nmpc.   set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')
classic.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

# initial reference velocity
velocity_reference = [0.1, 0.1, 0.0]

# Pattern Generator Event Loop
for i in range(160):
    print 'iteration: ', i
    # change reference velocities
    if 50 <= i < 100:
        velocity_reference = [ 0.1, -0.1, 0.0]
    if 100 <= i < 130:
        velocity_reference = [ 0.0,-0.2,-0.0]
    if 130 <= i:
        velocity_reference = [ 0.3, 0.0, 0.0]

    # set reference velocities to zero
    nmpc.   set_velocity_reference(velocity_reference)
    classic.set_velocity_reference(velocity_reference)

    # solve QP
    nmpc.   solve()
    classic.solve()

    # initial value embedding by internal states and simulation
    comx, comy, comz, footx, footy, footq, foot, comq= \
    nmpc.update()
    nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)
    if show_canvas:
        nmpc_p.update()

    comx, comy, comz, footx, footy, footq, foot, comq= \
    classic.update()
    classic.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)
    if show_canvas:
        classic_p.update()

nmpc.   data.save_to_file('./nmpc.json')
classic.data.save_to_file('./classic.json')

show_canvas  = False
save_to_file = True

nmpc_p    = Plotter(
    generator=None, show_canvas=show_canvas, save_to_file=save_to_file,
    filename='./nmpc',    fmt='pdf'
)
classic_p = Plotter(
    generator=None, show_canvas=show_canvas, save_to_file=save_to_file,
    filename='./classic', fmt='pdf'
)

nmpc_p   .load_from_file('./nmpc.json')
classic_p.load_from_file('./classic.json')

nmpc_p   .update()
classic_p.update()

nmpc_p   .create_data_plot()
classic_p.create_data_plot()
