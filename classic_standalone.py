import os, sys
import time
import numpy
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from walking_generator.visualization import Plotter
from walking_generator.classic import ClassicGenerator
from walking_generator.interpolation import Interpolation

# instantiate pattern generator
classic = ClassicGenerator(fsm_state='L/R')

# Pattern Generator Preparation
classic.set_security_margin(0.09, 0.05)

# instantiate plotter
show_canvas  = True
save_to_file = False
# classic_p = Plotter(classic, show_canvas, save_to_file)

# set initial values
comx = [0.00949035, 0.0, 0.0]
comy = [0.095,      0.0, 0.0]
comz = 0.814
footx = 0.00949035
footy = 0.095
footq = 0.0

classic.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

interpolClassic = Interpolation(0.005,classic)
# initial reference velocity
velocity_reference = [0.2, 0.0,0.2]

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
    classic.set_velocity_reference(velocity_reference)

    # solve QP
    classic.solve()
    classic.simulate()
    interpolClassic.interpolate(time)

    # initial value embedding by internal states and simulation
    comx, comy, comz, footx, footy, footq, foot, comq= \
    classic.update()
    classic.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)
    if show_canvas:
        classic_p.update()

classic.data.save_to_file('./classic_alone.json')

show_canvas  = False
save_to_file = True

# classic_p = Plotter(
#     generator=None, show_canvas=show_canvas, save_to_file=save_to_file,
#     filename='./classic_alone', fmt='pdf'
# )
classic_p.load_from_file('./classic_alone.json')
classic_p.update()
classic_p.create_data_plot()

interpolClassic.save_to_file("./classic_alone.csv")
