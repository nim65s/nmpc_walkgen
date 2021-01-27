import numpy as np

f = open("demofile.dat", "w")
f.write(str(2) + "1 2 3 4 5 \n")
f.close()

f = open("demofile.dat", "a")
f.write("1 2 3 6 5 \n")
f.close()

human_traj = np.loadtxt("demofile.dat")
print(human_traj)