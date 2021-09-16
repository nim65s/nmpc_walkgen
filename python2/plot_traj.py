import matplotlib.pyplot as plt
import numpy as np

path = '/local/imaroger/catkin_ws/src/trajectory_generation/data/DdpResult/DdpResult_from_-3.962,1.141,1.57_to_0,0,1.57_pos.dat'
traj = np.transpose(np.loadtxt(path))

plt.subplot(1,2,1)
plt.plot(traj[0],traj[1])

plt.subplot(1,2,2)
plt.plot(np.linspace(0,1,len(traj[2])),traj[2],linestyle=':', marker='o')
plt.show()