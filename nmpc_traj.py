import os, sys
import time
import numpy 
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from walking_generator.visualization_traj import Plotter
from walking_generator.combinedqp_traj import NMPCGeneratorTraj
from walking_generator.interpolation import Interpolation

from math import sqrt,floor
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# Load reference trajectory
path = '/local/imaroger/catkin_ws/src/trajectory_generation/data/Clothoid/Clothoid_from_0,0,-1.58_to_2,3,1.57_0.1_pos.dat'
traj = numpy.transpose(numpy.loadtxt(path))

traj_length = len(traj[0])

d_tot = 0
for i in range(1,traj_length):
    d_tot += sqrt((traj[0][i]-traj[0][i-1])**2 + (traj[1][i]-traj[1][i-1])**2)
    # print(sqrt((traj[0][i]-traj[0][i-1])**2 + (traj[1][i]-traj[1][i-1])**2))

velocity_ref = 0.3 # velocity we want the robot to walk
t_tot = d_tot/velocity_ref # total time necessary to walk the trajectory according the average velocity
new_length = int(16*(floor(t_tot/1.6)+1))
true_vel = d_tot/(new_length/16*1.6)
print(len(traj[0]),d_tot,t_tot,new_length,true_vel)

x,y,theta = traj[0],traj[1],traj[2]

okay = numpy.where(numpy.abs(numpy.diff(x)) + numpy.abs(numpy.diff(y)) > 0)
x,y = x[okay],y[okay]
tck, u = splprep([x, y], s=0)
unew = numpy.linspace(0,1,new_length)
data = splev(unew, tck)
x,y = data[0],data[1]

theta = numpy.interp(unew,numpy.linspace(0,1,traj_length),theta)

# plt.plot(traj[0],traj[1])
# plt.plot(x,y,linestyle=':', marker='o')
# plt.show()
# plt.plot(numpy.linspace(0,1,traj_length),traj[2])
# plt.plot(unew,theta,linestyle=':', marker='o')
# plt.show()

for i in range(0,new_length/16):
    print(len(x[i+15*i:i+16+15*i]),i+15*i,i+15+15*i)
    # plt.plot(traj[0],traj[1])
    # plt.plot(x[i+15*i:i+16+15*i],y[i+15*i:i+16+15*i],linestyle=':', marker='o')
    # plt.show()    

# # instantiate pattern generator
# nmpc    = NMPCGeneratorTraj(fsm_state='L/R')

# # Pattern Generator Preparation
# nmpc.   set_security_margin(0.09, 0.05)

# # instantiate plotter
# show_canvas  = True
# save_to_file = False
# nmpc_p    = Plotter(nmpc, trajectory_reference, show_canvas, save_to_file)

# # set initial values
# comx = [0.00949035, 0.0, 0.0]
# comy = [0.095,      0.0, 0.0]
# comz = 0.814
# footx = 0.00949035
# footy = 0.095
# footq = 0.0

# nmpc.   set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

# interpolNmpc = Interpolation(0.005,nmpc)

# # initial reference velocity
# trajectory_reference = 
# # velocity_reference = [0.2, 0.0,0.2]

# # Pattern Generator Event Loop
# for i in range(220):
#     print 'iteration: ', i
#     time = i*0.1

#     # # change reference velocities
#     # if 25 <= i < 50:
#     #     velocity_reference = [ 0.2, 0.0, -0.2]
#     # if 50 <= i < 150:
#     #     velocity_reference = [0.1, 0.2,-0.4]
#     # if 150 <= i < 200:
#     #     velocity_reference = [ 0.0, 0.2, 0.0]
#     # if 200 <= i :
#     #     velocity_reference = [ 0.0, 0.0, 0.0]

#     # # set reference velocities to zero
#     # nmpc.   set_velocity_reference(velocity_reference)

#     nmpc.   set_trajectory_reference(trajectory_reference)

#     # solve QP
#     nmpc.   solve()
#     nmpc.   simulate()
#     interpolNmpc.interpolate(time)

#     # initial value embedding by internal states and simulation
#     comx, comy, comz, footx, footy, footq, foot, comq= \
#     nmpc.update()
#     nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)
#     if show_canvas:
#         nmpc_p.update()

# nmpc.   data.save_to_file('./nmpc_alone.json')

# show_canvas  = False
# save_to_file = True

# nmpc_p    = Plotter(
#     generator=None, show_canvas=show_canvas, save_to_file=save_to_file,
#     filename='./nmpc_alone',    fmt='pdf'
# )

# nmpc_p   .load_from_file('./nmpc_alone.json')
# nmpc_p   .update()
# nmpc_p   .create_data_plot()

# interpolNmpc.save_to_file("./nmpc_alone.csv")
