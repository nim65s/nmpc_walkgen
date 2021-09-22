from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from math import cos,sin

data = np.transpose(np.loadtxt("../data/nmpc_vel_cython.dat"))

time = data[0]
com_x = data[1]
com_y = data[4]
zmp_x = data[16]
zmp_y = data[17]
support_foot_x = data[13]
support_foot_y = data[14]
states = data[-1]
foot = data[-2]

i, sum_foot = 7, 0
for i in range(7,len(states)):
	sum_foot += abs(foot[i])
	if sum_foot == 8:
		sum_foot = 0
		states[i] = 0
	i += 1


plt.plot(time,com_x,label = 'com_x')
plt.plot(time,com_y,label = 'com_y')
plt.plot(time,zmp_x,label = 'zmp_x')
plt.plot(time,zmp_y,label = 'zmp_y')
plt.plot(time,support_foot_x,label = 'foot_x')
plt.plot(time,support_foot_y,label = 'foot_y')
plt.scatter(time,states,label = 'phase',color='black')
plt.legend()
plt.show()

plt.plot(com_x,com_y,label = 'com')
plt.plot(zmp_x,zmp_y,label = 'zmp')
plt.plot(support_foot_x,support_foot_y,label='foot')
plt.legend()
plt.show()

# # # plt.plot(time_interp, traj[0])
# # # plt.plot(time_interp, traj[3])
# # # plt.plot(time_com, com[0])
# # # plt.plot(time_com, com[1])

# # # plt.show()

# # # plt.plot(time_interp, traj[14])
# # # plt.plot(time_foot, footL[2])

# # # plt.show()

# com_file.close()
# am_file.close()
# phase_file.close()
# footR_file.close()
# footL_file.close()

# com = np.transpose(np.loadtxt("data/com.dat"))
# footR = np.transpose(np.loadtxt("data/rightFoot.dat"))
# footL = np.transpose(np.loadtxt("data/leftFoot.dat"))
# phase = np.transpose(np.loadtxt("data/phases.dat"))
# am = np.transpose(np.loadtxt("data/am.dat"))

# plt.plot(com[0],com[1],color = 'blue',label='CoM')
# plt.plot(footL[0],footL[1],color = 'green',label='Left Foot')
# plt.plot(footR[0],footR[1],color = 'red',label='Right Foot')
# legend = plt.legend()

# plt.show()

# # fig = plt.figure()
# # ax = fig.add_subplot(111, projection='3d')
# # ax.plot3D(com[0],com[1],com[2],color = 'blue',label='CoM')
# # ax.plot3D(footL[0],footL[1],footL[2],color = 'green',label='Left Foot')
# # ax.plot3D(footR[0],footR[1],footR[2],color = 'red',label='Right Foot')
# # legend = plt.legend()

# # plt.show()

# time_com = np.linspace(0,100,len(com[0]))
# time_foot = np.linspace(0,100,len(footR[0]))

# plt.plot(time_foot,footR[2])
# plt.plot(time_foot,footL[2])
# plt.plot(time_foot,phase)
# plt.show()
# plt.plot(time_foot,footL[0])
# plt.plot(time_foot,footR[0])
# plt.plot(time_foot,footL[1])
# plt.plot(time_foot,footR[1])
# plt.show()
# plt.plot(time_foot,np.arccos(footR[3]))
# plt.plot(time_foot,np.arccos(footL[3]))
# plt.show()

# # time_reduced = np.arange(0,len(footR[0]),100)
# # arrow_len = 0.1

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot3D(footL[0],footL[1],footL[2],color = 'green',label='Left Foot')
# ax.plot3D(footR[0],footR[1],footR[2],color = 'red',label='Right Foot')
# legend = plt.legend()

# # ax.quiver(np.array(footR[0])[time_reduced],np.array(footR[1])[time_reduced],\
# # 	np.array(footR[2])[time_reduced],np.array(footR[3])[time_reduced],\
# # 	np.array(footR[6])[time_reduced],0, length=arrow_len, lw = 2)
# # ax.quiver(np.array(footL[0])[time_reduced],np.array(footL[1])[time_reduced],\
# # 	np.array(footL[2])[time_reduced],np.array(footL[3])[time_reduced],\
# # 	np.array(footL[6])[time_reduced],0, length=arrow_len, lw = 2)

# # ax.quiver(np.array(com[0])[time_reduced],np.array(com[1])[time_reduced],\
# # 	np.array(com[2])[time_reduced],np.array(np.cos(am[0]))[time_reduced],\
# # 	np.array(np.sin(am[0]))[time_reduced],0, length=arrow_len, lw = 2)


# plt.show()


# # Check old versus new version
# # com_old = np.transpose(np.loadtxt("data/com_old.dat"))
# # footR_old = np.transpose(np.loadtxt("data/rightFoot_old.dat"))
# # am_old = np.transpose(np.loadtxt("data/am_old.dat"))

# # time_c = np.linspace(0,100,len(com[0]))
# # time_c_old = np.linspace(17,100,len(com_old[0]))

# # time_f = np.linspace(0,100,len(footR[0]))
# # time_f_old = np.linspace(17,100,len(footR_old[0]))

# # plt.plot(time_c,com[0])
# # plt.plot(time_c_old,com_old[0])

# # plt.show()

# # plt.plot(time_c,am[0])
# # plt.plot(time_c_old,am_old[0])

# # plt.show()

# # plt.plot(time_f,footR[0])
# # plt.plot(time_f_old,footR_old[0])

# # plt.show()
# # plt.plot(time_f,footR[2])
# # plt.plot(time_f_old,footR_old[2])

# # plt.show()

# # plt.plot(time_f,footR[3])
# # plt.plot(time_f_old,footR_old[3])

# # plt.show()