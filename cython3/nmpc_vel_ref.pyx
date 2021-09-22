from wpg.combinedqp cimport NMPCGenerator
import time
import numpy as np
cimport numpy as np
import os, sys
# from cpython cimport array
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from qpoases import PySQProblem as SQProblem
from qpoases import PySolutionAnalysis as SolutionAnalysis





cpdef public int nmpc_vel_ref() except -1:



    cdef np.ndarray comx, comy, comq, velocity_reference
    cdef float comz, footx, footy, footq
    cdef int nb_step
    cdef str foot = 'left'
    cdef str state = 'D'

    nmpc = NMPCGenerator(fsm_state=state)

    nmpc.set_security_margin(0.09,0.05)

    velocity_reference = np.array([0.,0.,0.])

    comx = np.array([-3.16e-3, 0.0, 0.0])
    comy = np.array([1.237384291203724555e-03,0.0, 0.0])
    comz = 8.786810585901939641e-01
    comq = np.array([0.,0.,0.])
    footx = 1.86e-4
    footy = 0.085
    footq = 0.0 

    nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)

    nb_step = 10    

    f = open("../data/nmpc_vel_cython.dat", "w")
    f.write("")
    f.close()

    cdef qp = SQProblem(nmpc.nv,nmpc.nc)
    options = Options()
    options.setToMPC()
    options.printLevel = PrintLevel.LOW
    qp.setOptions(options)

    for i in range(8*nb_step):
        # print("iteration : ",i)

        if 7 <= i < 8*(nb_step-2)-1 :
            velocity_reference = np.array([0.2, 0., 0.])
        if 8*(nb_step-2)-1 <= i:
            velocity_reference = np.array([0., 0., 0.])

        nmpc.set_velocity_reference(velocity_reference)

        nmpc.preprocess_solution()

        # solve
        nmpc.cputime = np.array([2.9]) # ms
        nmpc.nwsr = np.array([1000]) # unlimited bounded

        if not nmpc._qp_is_initialized:
            qp.init(
                nmpc.qp_H, nmpc.qp_g, nmpc.qp_A,
                nmpc.qp_lb, nmpc.qp_ub,
                nmpc.qp_lbA, nmpc.qp_ubA,
                nmpc.nwsr, nmpc.cputime
            )
            nwsr, cputime = nmpc.nwsr, nmpc.cputime
            nmpc._qp_is_initialized = True
        else:
            qp.hotstart(
                nmpc.qp_H, nmpc.qp_g, nmpc.qp_A,
                nmpc.qp_lb, nmpc.qp_ub,
                nmpc.qp_lbA, nmpc.qp_ubA,
                nmpc.nwsr, nmpc.cputime
            )
            nwsr, cputime = nmpc.nwsr, nmpc.cputime

        # orientation primal solution
        qp.getPrimalSolution(nmpc.dofs)

        # save qp solver data
        nmpc.qp_nwsr    = nwsr          # working set recalculations
        nmpc.qp_cputime = cputime*1000. # in milliseconds (set to 2.9ms)        

        nmpc.postprocess_solution()

        nmpc.simulate()

        comx, comy, comz, footx, footy, footq, foot, comq, state = nmpc.update()

        nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)

        zmpx = comx[0]-comz/9.81*comx[2]
        zmpy = comy[0]-comz/9.81*comy[2]  

        if state == 'D':
            state_bool = 0
        elif state == 'L':
            state_bool = 1
        else:
            state_bool = -1
        if foot == 'left':
            foot_bool = 1
        else :
            foot_bool = -1

        print(nmpc.fsm_states,foot,foot_bool,state_bool)

        f = open("../data/nmpc_vel_cython.dat", "a")
        line = str(time.time()) + " " + str(comx[0])+ "  " + str(comx[1])+ "  " + str(comx[2])+ "  " +\
            str(comy[0])+ "  " + str(comy[1])+ "  " + str(comy[2])+ "  " +\
            str(comz)+ "  0  0  " + str(comq[0]) + "  " + str(comq[1]) + "  " +\
            str(comq[2]) + "  " + str(footx) + "  " + str(footy)+ "  " +\
            str(footq) +  "  " + str(zmpx) + "  " + str(zmpy) + "  " + str(foot_bool) \
            + "  " + str(state_bool) + " \n"
        f.write(line)
        f.close()

    return 0

