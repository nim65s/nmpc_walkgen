from walking_generator.visualization import Plotter
from walking_generator.classic import ClassicGenerator

# instantiate pattern generator
gen = ClassicGenerator(fsm_state='L/R')

# instantiate plotter
show_canvas = True
save_to_file = False
plot = Plotter(gen, show_canvas, save_to_file)

# Pattern Generator Preparation
# set reference velocities to zero
gen.dC_kp1_x_ref[...] = 0.2
gen.dC_kp1_y_ref[...] = 0.0
gen.dC_kp1_q_ref[...] = 0.0

# set initial values
comx = [0.06591456,0.07638739,-0.1467377]
comy = [2.49008564e-02,6.61665254e-02,6.72712187e-01]
comz = 0.814
footx = 0.00949035
footy = 0.095
footq = 0.0
gen.set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

gen.simulate()
gen._update_data()

# Pattern Generator Event Loop
for i in range(20):
    print 'iteration: ', i
    # solve QP
    gen.solve()

    # initial value embedding by internal states and simulation
    #gen.update()
    plot.update()

    raw_input('press key:')

gen.data.save_to_file('./data.json')
