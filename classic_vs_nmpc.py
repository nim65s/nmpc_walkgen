import os, sys
import time
import numpy
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from walking_generator.visualization import Plotter
from walking_generator.classic import ClassicGenerator
from walking_generator.combinedqp import NMPCGenerator
from walking_generator.interpolation import Interpolation

# instantiate pattern generator
nmpc    = NMPCGenerator(fsm_state='L/R')
classic = ClassicGenerator(fsm_state='L/R')

# Pattern Generator Preparation
nmpc.   set_security_margin(0.09, 0.05)
classic.set_security_margin(0.09, 0.05)

# instantiate plotter
show_canvas  = False
save_to_file = True
fmt='jpeg'
dpi=200
nmpc_p    = Plotter(nmpc,    show_canvas=show_canvas, save_to_file=save_to_file,
        filename='./nmpc/nmpc.png', fmt=fmt, dpi=dpi
)
classic_p = Plotter(classic, show_canvas=show_canvas, save_to_file=save_to_file,
        filename='./classic/classic.png', fmt=fmt, dpi=dpi
)

# set initial values
comx = [0.00949035, 0.0, 0.0]
comy = [0.095,      0.0, 0.0]
comz = 0.814
footx = 0.00949035
footy = 0.095
footq = 0.0

nmpc.   set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')
classic.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

interpolClassic = Interpolation(0.005,classic)
interpolNmpc = Interpolation(0.005,nmpc)

# initial reference velocity
velocity_reference = [0.2, 0.0, 0.2]

# Pattern Generator Event Loop
for i in range(220):
    print 'iteration: ', i
    time = i*0.1

    # change reference velocities
    if 25 <= i < 50:
        velocity_reference = [ 0.2, 0.0, -0.2]
    if 50 <= i < 150:
        velocity_reference = [0.1, 0.2,-0.4]
    if 150 <= i < 200:
        velocity_reference = [ 0.0, 0.2, 0.0]
    if 200 <= i :
        velocity_reference = [ 0.0, 0.0, 0.0]

    # set reference velocities to zero
    nmpc.   set_velocity_reference(velocity_reference)
    classic.set_velocity_reference(velocity_reference)

    # solve QP
    nmpc.   solve()
    classic.solve()
    interpolClassic.interpolate(time)
    interpolNmpc.interpolate(time)

    # initial value embedding by internal states and simulation
    comx, comy, comz, footx, footy, footq, foot, comq= \
    nmpc.update()
    nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)
    if show_canvas or save_to_file:
        nmpc_p.update()

    comx, comy, comz, footx, footy, footq, foot, comq= \
    classic.update()
    classic.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)
    if show_canvas or save_to_file:
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

interpolClassic.save_to_file("./wieber2010python.csv")
interpolNmpc.save_to_file("./NMPCpython.csv")
