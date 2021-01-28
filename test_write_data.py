import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
# f = open("demofile.dat", "w")
# f.write(str(2) + "1 2 3 4 5 \n")
# f.close()

# f = open("demofile.dat", "a")
# f.write("1 2 3 6 5 \n")
# f.close()

# f = open("demofile.dat", "w")
# f.write(str(2) + "1 2 3 4 5 \n")
# f.write(str(2) + "1 2 3 0 5 \n")
# f.close()


traj = np.transpose(np.loadtxt("nmpc_traj.csv"))
print(len(traj),len(traj[0]))

time = np.linspace(0,100, len(traj[0]))


# plt.plot(time,traj[13])
# plt.plot(time,traj[17])
# plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(traj[0],traj[3],traj[6],color = 'blue',label='CoM')
ax.plot3D(traj[12],traj[13],traj[14],color = 'green',label='Left Foot')
ax.plot3D(traj[16],traj[17],traj[18],color = 'red',label='Right Foot')
legend = plt.legend()

plt.show()
# for i in range(len(traj[14])):
# 	if traj[14][i] != 0:
# 		print(time[i],traj[14][i])

# plt.plot(time,traj[14])
# # plt.plot(time,traj[18])
# plt.show()