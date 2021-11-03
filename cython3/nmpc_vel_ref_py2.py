import os, sys
import time
import numpy
# numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from walking_generator.visualization import Plotter
from walking_generator.combinedqp import NMPCGenerator
# from walking_generator.combinedqp_init import NMPCGeneratorInit
from walking_generator.interpolation import Interpolation

# instantiate pattern generator
nmpc    = NMPCGenerator(fsm_state='D')

# Pattern Generator Preparation
nmpc.   set_security_margin(0.09, 0.05)

# instantiate plotter
show_canvas  = True
save_to_file = False
nmpc_p    = Plotter(nmpc,    show_canvas, save_to_file)

# set initial values
comx = [-3.16e-3, 0.0, 0.0]#[0.00679821, 0.0, 0.0]
comy = [1.237384291203724555e-03,0.0, 0.0] #[0.08693283,0.0, 0.0] #0.03
comz = 8.786810585901939641e-01 
footx = 1.86e-4#0.00949035#-0.008
footy = 0.085
footq = 0.0
# Fx = [0.00949035,0.00949035]
# Fy = [0.095,-0.095]
# Fq = [0.0,0.0]
nmpc.   set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

interpolNmpc = Interpolation(0.001,nmpc)
# initial reference velocity
velocity_reference = [0., 0.,0.0]

f = open("../data/nmpc_vel.dat", "w")
f.write("")
f.close()

time_list = []

nb_step = 10
# raw_input()
# Pattern Generator Event Loop
for i in range (8*nb_step):
    print 'iteration: ', i
    time_iter = i*0.1

    # if 7 <= i < 8*4-1 :
    #     velocity_reference = [0.2, 0.0, 0.0]
    # if 8*4-1 <= i < 8*15-1 :
    #     velocity_reference = [0.1, 0., 0.2]
    # if 8*15-1 <= i < 8*22-1 :
    #     velocity_reference = [0.2, 0.0, 0.1]
    # if 8*22-1 <= i < 8*(nb_step-2)-1 :
    #     velocity_reference = [0.2, 0.0, -0.1]  
    # if 7 <= i < 8*5-1 :
    #     velocity_reference = [0.2, 0., 0.]
    # if 8*5-1 <= i < 8*9-1 :
    #     velocity_reference = [0.2, 0., 0.1]        
    # if 8*9-1 <= i < 8*14-1 :
    #     velocity_reference = [0., 0.04, 0.]
    if 7 <= i < 8*(nb_step-2)-1 :
        velocity_reference = [0.2, 0., 0.]

    if 8*(nb_step-2)-1 <= i:
        velocity_reference = [0.0, 0.0, 0.0]
    # print("vel : ",velocity_reference)
    start_time = time.time()
    # set reference velocities to zero
    nmpc.   set_velocity_reference(velocity_reference)

    # solve QP
    nmpc.   solve()
    nmpc.   simulate()
    interpolNmpc.interpolate(time_iter)

    # initial value embedding by internal states and simulation
    comx, comy, comz, footx, footy, footq, foot, comq, state = \
    nmpc.update()
    # print(nmpc.fsm_states,foot)
    # print(comx[0],nmpc.C_kp1_x[0],nmpc.C_kp1_x[-1],interpolNmpc.curleft.x,interpolNmpc.CoMbuffer[-1].x)
    nmpc.set_initial_values(comx, comy, comz, footx, footy, footq,\
    foot, comq)
    time_list.append(time.time() - start_time)
    zmpx = comx[0]-comz/9.81*comx[2]
    zmpy = comy[0]-comz/9.81*comy[2]  

    if state == 'D':
        state_bool = 0
    elif state == 'L/R':
        state_bool = 1
    else:
        state_bool = -1
    if foot == 'left':
        foot_bool = 1
    else :
        foot_bool = -1

    f = open("../data/nmpc_vel.dat", "a")
    line = str(time.time()) + " " + str(comx[0])+ "  " + str(comx[1])+ "  " + str(comx[2])+ "  " +\
        str(comy[0])+ "  " + str(comy[1])+ "  " + str(comy[2])+ "  " +\
        str(comz)+ "  0  0  " + str(comq[0]) + "  " + str(comq[1]) + "  " +\
        str(comq[2]) + "  " + str(footx) + "  " + str(footy)+ "  " +\
        str(footq) +  "  " + str(zmpx) + "  " + str(zmpy) + "  " + str(foot_bool) \
        + "  " + str(state_bool) + " \n"
    f.write(line)
    f.close()


    if show_canvas:
        nmpc_p.update()
        
print(len(time_list),numpy.mean(time_list))

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
