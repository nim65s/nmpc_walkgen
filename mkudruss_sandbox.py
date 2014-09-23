import time
from walking_generator.visualization import Plotter
from walking_generator.classic import ClassicGenerator

# instantiate pattern generator
gen = ClassicGenerator(fsm_state='L/R')

# instantiate plotter
show_canvas = False
save_to_file = False
plot = Plotter(gen, show_canvas, save_to_file)

# Pattern Generator Preparation
# set reference velocities to zero
gen.dC_kp1_x_ref[...] = 0.2
gen.dC_kp1_y_ref[...] = 0.0
gen.dC_kp1_q_ref[...] = 0.0

# set initial values
comx = [0.00949035,0.0,0.0]
comy = [0.095,0.0,0.0]
comz = 0.814
footx = 0.00949035
footy = 0.095
footq = 0.0
gen.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

gen.simulate()
gen._update_data()

# Pattern Generator Event Loop
for i in range(50):
    print 'iteration: ', i
    # solve QP
    gen.solve()

    # initial value embedding by internal states and simulation
    comx, comy, comz, footx, footy, footq, foot, comq= \
    gen.update()
    gen.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)
    plot.update()

    #raw_input('press key:')
    time.sleep(0.1)

gen.data.save_to_file('./data.json')

