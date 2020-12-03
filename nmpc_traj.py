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

def resizeTraj(traj,velocity_ref,velocity_low):
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

    ind = numpy.where(numpy.abs(numpy.diff(theta))>0.1)
    print(ind)

    ind_partition, d  = [[0]], []
    i,previous = 0,"ref"
    while i < traj_length-1:
        if numpy.sum(numpy.isin(ind,i)) == 0:
            if previous == "ref":
                ind_partition[-1].append(i+1) 
                # d[-1] += sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2)
                # print("1",i+1,i)
            else:          
                ind_partition.append([])  
                # d.append(sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2))           
                ind_partition[-1].append(i)
                ind_partition[-1].append(i+1)
                # print("2",i+1,i)
            previous = "ref"
            i+=1
        else:
            if previous == "ref":
                ind_partition.append([])
                # d.append(sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2))                   
                ind_partition[-1].append(i)
                ind_partition[-1].append(i+1)
                # ind_partition[-1].append(i+2)
                # ind_partition[-1].append(i+3)
                # ind_partition[-1].append(i+4)     
                # ind_partition[-1].append(i+5)                

                # print("3",i+1,i)
            else:    
                ind_partition[-1].append(i+1)                             
                # d[-1] += sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2)    
                # print("4",i+1,i)
            previous = "low"
            i+=1

        # i += 1

    print(ind_partition)
    new_length_list = []

    for k in range(len(ind_partition)):
        d.append(0)
        for i in ind_partition[k][:-1]:
            # print(i)
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
            # print(ind,len(x),len(theta))
            current_x,current_y,current_theta = x[ind],y[ind],theta[ind]
            # print(current_x,current_y,length)                     

            # okay = numpy.where(numpy.abs(numpy.diff(current_x)) + numpy.abs(numpy.diff(current_y)) > 0)
            # print(okay)
            # current_x,current_y = current_x[okay],current_y[okay]
            # print(current_x,current_y) 
            # tck, u = splprep([current_x, current_y], s=0)
            # unew = numpy.linspace(0,1,length)
            # data = splev(unew, tck)
            # current_x,current_y = data[0],data[1]
            new_time = numpy.linspace(0,1,length)
            old_time = numpy.linspace(0,1,len(ind))
            current_x = numpy.interp(new_time,old_time,current_x) 
            current_y = numpy.interp(new_time,old_time,current_y)              
            current_theta = numpy.interp(new_time,old_time,current_theta)  

            new_x = numpy.concatenate((new_x,current_x))
            new_y = numpy.concatenate((new_y,current_y))
            new_theta = numpy.concatenate((new_theta,current_theta))

            # print(current_x)
        i += 1

    # print(new_x)



    # d_vref = 0

    # d_tot = 0
    # for i in range(0,traj_length-1):
    #     if numpy.sum(numpy.isin(ind,i)) == 0:
    #         d_vref += sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2)
    # print(d_vref)



        
    #     # print(sqrt((traj[0][i]-traj[0][i-1])**2 + (traj[1][i]-traj[1][i-1])**2))

    # t_tot = d_vref/velocity_ref # total time necessary to walk the trajectory according the average velocity
    # new_length = int((floor(t_tot/0.1)))
    # true_vel = d_tot/(new_length*0.1)
    # # print(len(traj[0]),d_tot,t_tot,new_length,int(16*(floor(t_tot/1.6)+1)),true_vel)





    # okay = numpy.where(numpy.abs(numpy.diff(x)) + numpy.abs(numpy.diff(y)) > 0)
    # x,y = x[okay],y[okay]
    # tck, u = splprep([x, y], s=0)
    # unew = numpy.linspace(0,1,new_length)
    # data = splev(unew, tck)
    # x,y = data[0],data[1]

    # theta = numpy.interp(unew,numpy.linspace(0,1,traj_length),theta)    

    # plt.plot(traj[0],traj[1])
    # plt.plot(new_x,new_y,linestyle=':', marker='o')
    # plt.show()

    # time = numpy.linspace(0,1,traj_length)
    # plt.plot(time,traj[2])
    # plt.plot(numpy.linspace(0,1,len(new_theta)),new_theta,linestyle=':', marker='o')
    # plt.show()

    # time_diff = numpy.linspace(0,1,len(numpy.diff(x)))
    # plt.plot(time_diff,numpy.sqrt(numpy.diff(x)**2+numpy.diff(y)**2))
    # plt.show()

    # plt.plot(time_diff,numpy.diff(theta))
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

# def resizeTraj(traj,velocity_ref):
#     traj_length = len(traj[0])
#     x,y,theta = traj[0],traj[1],traj[2]
#     d_tot = 0
#     for i in range(0,traj_length-1):
#         d_tot += sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2)
    
#     t_tot = d_tot/velocity_ref # total time necessary to walk the trajectory according the average velocity
#     new_length = int((floor(t_tot/0.1)))
#     true_vel = d_tot/(new_length*0.1)
#     # print(len(traj[0]),d_tot,t_tot,new_length,int(16*(floor(t_tot/1.6)+1)),true_vel)

#     okay = numpy.where(numpy.abs(numpy.diff(x)) + numpy.abs(numpy.diff(y)) > 0)
#     x,y = x[okay],y[okay]
#     tck, u = splprep([x, y], s=0)
#     unew = numpy.linspace(0,1,new_length)
#     data = splev(unew, tck)
#     x,y = data[0],data[1]

#     theta = numpy.interp(unew,numpy.linspace(0,1,traj_length),theta)    

#     # plt.plot(traj[0],traj[1])
#     # plt.plot(x,y,linestyle=':', marker='o')
#     # plt.show()

#     # time = numpy.linspace(0,1,traj_length)
#     # plt.plot(time,traj[2])
#     # plt.plot(numpy.linspace(0,1,len(theta)),theta,linestyle=':', marker='o')
#     # plt.show()

#     # time_diff = numpy.linspace(0,1,len(numpy.diff(x)))
#     # plt.plot(time_diff,numpy.sqrt(numpy.diff(x)**2+numpy.diff(y)**2))
#     # plt.show()

#     # plt.plot(time_diff,numpy.diff(theta))
#     # plt.show()

#     new_traj = numpy.zeros((3,new_length), dtype=float)
#     new_traj[0],new_traj[1],new_traj[2] = x,y,theta
#     return new_traj



# Load reference trajectory
# path = '/local/imaroger/catkin_ws/src/trajectory_generation/data/Clothoid/Clothoid_from_0,0,-1.58_to_1,2,1.57_0.1_pos.dat' #Clothoid
name = 'DdpResult_from_-4.11,-0.68,0.0_to_0,0,1.57_pos'
path = '/local/imaroger/catkin_ws/src/trajectory_generation/data/DdpResult/'+name+'.dat'
traj = numpy.transpose(numpy.loadtxt(path))

# plt.subplot(1,2,1)
# plt.plot(traj[0],traj[1])

traj = translate(traj)
# print(traj[2])
# plt.subplot(1,2,2)
# plt.plot(traj[0],traj[1])
# plt.show()


velocity_ref = 0.2 # velocity we want the robot to walk
velocity_low = 0.0005 # velocity we want the robot to walk

# resized_traj = numpy.flip(resizeTraj(traj, velocity_ref),1)   
# resized_traj = resizeTraj(traj, velocity_ref)
resized_traj = resizeTraj(traj, velocity_ref,velocity_low)

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
