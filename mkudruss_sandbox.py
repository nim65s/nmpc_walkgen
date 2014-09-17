from walking_generator.heler import Plotter
from walking_generator.classic import ClassicGenerator

# instantiate pattern generator
gen = ClassicGenerator(fsm_state='L/R')

# instantiate plotter
show_canvas = True
save2file = False
plot = Plotter(gen, show_canvas, save2file)


# Pattern Generator Preparation
# set initial values
comx = [0.06591456,0.07638739,-0.1467377]
comy = [2.49008564e-02,6.61665254e-02,6.72712187e-01]
comz = 0.814
footx = 0.00949035
footy = 0.095
footq = 0.0
gen.set_initial_state(comx, comy, comz, footx, footy, footq, foot='left')

# set reference velocities to zero
gen.dC_kp1_x_ref[...] = 0.2
gen.dC_kp1_y_ref[...] = 0.0
gen.dC_kp1_q_ref[...] = 0.0

# Pattern Generator Event Loop
for i in range(10):
    print 'iteration: ', i
    # solve QP
    gen.solve()

    # update states by simulation
    gen.simulate()

    # initial value embedding
    gen.c_k_x[0] = gen.  C_kp1_x[0]
    gen.c_k_x[1] = gen. dC_kp1_x[0]
    gen.c_k_x[2] = gen.ddC_kp1_x[0]
    gen.c_k_y[0] = gen.  C_kp1_y[0]
    gen.c_k_y[1] = gen. dC_kp1_y[0]
    gen.c_k_y[2] = gen.ddC_kp1_y[0]
    gen.c_k_q[0] = gen.  C_kp1_q[0]
    gen.c_k_q[1] = gen. dC_kp1_q[0]
    gen.c_k_q[2] = gen.ddC_kp1_q[0]

    raw_input('press return to continue: ')

