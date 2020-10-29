import numpy as np
from qpoases import PySQProblem as SQProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel

#Setup data of first QP.

H   = np.array([1.0, 0.0, 0.0, 0.5 ]).reshape((2,2))
A   = np.array([1.0, 1.0 ]).reshape((2,1))
g   = np.array([1.5, 1.0 ])
lb  = np.array([0.5, -2.0])
ub  = np.array([5.0, 2.0 ])
lbA = np.array([-1.0 ])
ubA = np.array([2.0])


# Setup data of second QP.

g_new   = np.array([1.0, 1.5])
lb_new  = np.array([0.0, -1.0])
ub_new  = np.array([5.0, -0.5])
lbA_new = np.array([-2.0])
ubA_new = np.array([1.0])


# Setting up SQProblem object.

example = SQProblem(2, 1)
options = Options()
options.setToMPC()
#options.printLevel = PrintLevel.NONE
example.setOptions(options)

# Solve first QP.
nWSR = np.array([10])
example.init(H, g, A, lb, ub, lbA, ubA, nWSR)
print("---------", nWSR)

xOpt = np.zeros(2)
example.getPrimalSolution(xOpt)
print("\nxOpt = [ %e, %e ];  objVal = %e\n\n"%(xOpt[0],xOpt[1],example.getObjVal()))

# Solve second QP.
nWSR = np.array([10])
example.hotstart(H, g_new,A, lb_new, ub_new, lbA_new, ubA_new, nWSR)

# Get and print solution of second QP.

example.getPrimalSolution(xOpt)
print("\nxOpt = [ %e, %e ];  objVal = %e\n\n"%(xOpt[0],xOpt[1],example.getObjVal()))
example.printOptions()