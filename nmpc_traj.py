import os, sys
import time
import numpy 
numpy.set_printoptions(threshold=numpy.nan, linewidth =numpy.nan)
from walking_generator.visualization_traj import PlotterTraj
from walking_generator.combinedqp_traj import NMPCGeneratorTraj
from walking_generator.interpolation import Interpolation

from math import sqrt,floor
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev


def resizeTraj1(traj,velocity_ref):
    traj_length = len(traj[0])
    x,y,theta = traj[0],traj[1],traj[2]

    # time_diff = numpy.linspace(0,1,len(numpy.diff(theta)))
    # plt.plot(time_diff,numpy.diff(theta))
    # plt.show()   

    okay = numpy.where(numpy.abs(numpy.diff(x)) + numpy.abs(numpy.diff(y)) > 0)
    x,y = x[okay],y[okay]
    tck, u = splprep([x, y], s=0)
    unew = numpy.linspace(0,1,traj_length)
    data = splev(unew, tck)
    x,y = data[0],data[1]

    # x2 = numpy.interp(numpy.linspace(0,1,20),unew,x)
    # y2 = numpy.interp(numpy.linspace(0,1,20),unew,y)

    # plt.plot(traj[0],traj[1],linestyle=':', marker='o')
    # plt.plot(x,y,linestyle=':', marker='o')   
    # plt.plot(x2,y2,linestyle=':', marker='o')
    # plt.show()

    print(traj_length,len(x),len(theta))

    ind = numpy.where(numpy.abs(numpy.diff(theta))>0.2)
    print(ind)

    max_delta_ori = numpy.max(numpy.abs(numpy.diff(theta)))
    if max_delta_ori < 0.8:
        velocity_low = 0.05
    elif max_delta_ori < 1.7:
        velocity_low = 0.001
    elif max_delta_ori < 2.8:
        velocity_low = 0.0005
    else:
        velocity_low = 0.0001

    print(max_delta_ori,velocity_low)

    ind_partition, d  = [[0]], []
    i,previous = 0,"ref"
    while i < traj_length-1:
        if numpy.sum(numpy.isin(ind,i)) == 0:
            if previous == "low":     
                ind_partition.append([])           
                ind_partition[-1].append(i)
            ind_partition[-1].append(i+1)
            previous = "ref"
            i+=1
        else:
            if previous == "ref":
                ind_partition.append([])              
                ind_partition[-1].append(i)
            ind_partition[-1].append(i+1)                           
            previous = "low"
            i+=1

    print(ind_partition)
    new_length_list = []

    for k in range(len(ind_partition)):
        d.append(0)
        for i in ind_partition[k][:-1]:
            d[-1] += sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2)
        if k%2 == 0:
            t = d[-1]/velocity_ref
        else:
            t = d[-1]/velocity_low
        new_length_list.append(int((floor(t/0.1))))        

    print(d)
    print(new_length_list)
    
    new_x,new_y,new_theta = numpy.array([]),numpy.array([]),numpy.array([])
    i = 0
    for length in new_length_list:
        if length != 0:
            ind = numpy.array(ind_partition[i])
            current_x,current_y,current_theta = x[ind],y[ind],theta[ind]

            new_time = numpy.linspace(0,1,length)
            old_time = numpy.linspace(0,1,len(ind))
            current_x = numpy.interp(new_time,old_time,current_x) 
            current_y = numpy.interp(new_time,old_time,current_y)              
            current_theta = numpy.interp(new_time,old_time,current_theta)  

            new_x = numpy.concatenate((new_x,current_x))
            new_y = numpy.concatenate((new_y,current_y))
            new_theta = numpy.concatenate((new_theta,current_theta))
        i += 1

    # plt.plot(traj[0],traj[1])
    # plt.plot(new_x,new_y,linestyle=':', marker='o')
    # plt.show()

    # time = numpy.linspace(0,1,traj_length)
    # new_time = numpy.linspace(0,1,len(new_theta))
    # plt.plot(time,traj[2])
    # # plt.plot(new_time,new_x)
    # plt.plot(new_time,new_theta,linestyle=':', marker='o')
    # plt.show()

    new_traj = numpy.zeros((3,len(new_x)), dtype=float)
    new_traj[0],new_traj[1],new_traj[2] = new_x,new_y,new_theta
    return new_traj

def translate(traj):
    x_0,y_0,theta_0 = traj[0][0],traj[1][0],traj[2][0]
    # print(traj[1]-y_0,traj[0]-x_0)
    traj[0],traj[1] = (traj[0]-x_0)*numpy.cos(theta_0) + (traj[1]-y_0)\
        *numpy.sin(theta_0), -(traj[0]-x_0)*numpy.sin(theta_0) + \
        (traj[1]-y_0)*numpy.cos(theta_0)
    # print(-(traj[0][0]-x_0)*numpy.sin(theta_0) + (traj[1][0]-y_0)*numpy.cos(theta_0))
    traj[2] = traj[2] - theta_0 
    return traj 


# Load reference trajectory
# path = '/local/imaroger/catkin_ws/src/trajectory_generation/data/Clothoid/Clothoid_from_0,0,-1.58_to_1,2,1.57_0.1_pos.dat' #Clothoid
name = 'DdpResult_from_-3.962,1.141,1.57_to_0,0,1.57_pos'
path = '/local/imaroger/catkin_ws/src/trajectory_generation/data/DdpResult/'+name+'.dat'
traj = numpy.transpose(numpy.loadtxt(path))

# plt.subplot(1,2,1)
# plt.plot(traj[0],traj[1])

traj = translate(traj)
# print(traj[2])
# plt.subplot(1,2,2)
# plt.plot(traj[0],traj[1])
# plt.show()


velocity_ref = 0.15 # velocity we want the robot to walk
# velocity_low = 0.001 # velocity we want the robot to walk

# resized_traj = numpy.flip(resizeTraj(traj, velocity_ref),1)   
# resized_traj = resizeTraj(traj, velocity_ref)
resized_traj = resizeTraj(traj, velocity_ref)

# instantiate pattern generator
nmpc    = NMPCGeneratorTraj(fsm_state='L/R')

# Pattern Generator Preparation
nmpc.   set_security_margin(0.09, 0.05)

# instantiate plotter
show_canvas  = True
save_to_file = False
nmpc_p    = PlotterTraj(nmpc, traj, show_canvas, save_to_file)

raw_input("Press Enter to start")

# set initial values
comx = [0.00949035, 0.0, 0.0]
comy = [0.095,0.0, 0.0]
comz = 0.814
footx = 0.00949035
footy = 0.095
footq = 0.0

nmpc.   set_initial_values(comx, comy, comz, footx, footy, footq, foot='left')

interpolNmpc = Interpolation(0.005,nmpc)

# initial reference velocity
# velocity_reference = [0.2, 0.0,0.2]

# Pattern Generator Event Loop
for i in range(16,len(resized_traj[0])):
    # print(i)
    trajectory_reference = resized_traj[:,i-16:i]
    time = (i-16)*0.1
    # print(trajectory_reference)

    # print(i-16,i,len(resized_traj[0]),len(trajectory_reference[0]))

    # plt.plot(traj[0],traj[1])
    # plt.plot(trajectory_reference[0],trajectory_reference[1],linestyle=':', marker='o')
    # plt.show() 

# for i in range(220):
#     print 'iteration: ', i
#     time = i*0.1

    # # change reference velocities
    # if 25 <= i < 50:
    #     velocity_reference = [ 0.2, 0.0, -0.2]
    # if 50 <= i < 150:
    #     velocity_reference = [0.1, 0.2,-0.4]
    # if 150 <= i < 200:
    #     velocity_reference = [ 0.0, 0.2, 0.0]
    # if 200 <= i :
    #     velocity_reference = [ 0.0, 0.0, 0.0]

    # # set reference velocities to zero
    # nmpc.   set_velocity_reference(velocity_reference)

    nmpc.   set_trajectory_reference(trajectory_reference)

    # solve QP
    nmpc.   solve()
    nmpc.   simulate()
    interpolNmpc.interpolate(time)

    # initial value embedding by internal states and simulation
    comx, comy, comz, footx, footy, footq, foot, comq= \
    nmpc.update()
    # print("------",comx, comy, comz, footx, footy, footq, foot, comq)
    nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)
    if show_canvas:
        nmpc_p.update()

nmpc.   data.save_to_file('./nmpc_traj.json')

show_canvas  = False
save_to_file = True

nmpc_p    = PlotterTraj(
    generator=None, trajectory=traj, show_canvas=show_canvas, save_to_file=save_to_file,
    filename='./nmpc_traj',    fmt='pdf'
)

nmpc_p   .load_from_file('./nmpc_traj.json')
nmpc_p   .update()
nmpc_p   .create_data_plot()

interpolNmpc.save_to_file("./nmpc_traj.csv")
