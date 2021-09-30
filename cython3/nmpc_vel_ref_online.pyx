from wpg.combinedqp cimport NMPCGenerator
from wpg.interpolation cimport Interpolation
import time
import numpy as np
cimport numpy as np
import os, sys
# from cpython cimport array
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from qpoases import PySQProblem as SQProblem
from qpoases import PySolutionAnalysis as SolutionAnalysis

import signal

class AlarmException(Exception):
    pass

def alarmHandler(signum, frame):
    raise AlarmException

def nonBlockingRawInput(previous,timeout):
    signal.signal(signal.SIGALRM, alarmHandler)
    signal.setitimer(signal.ITIMER_REAL,timeout,0.0)
    try:
        text = raw_input()
        signal.alarm(0)
        print("new vel !",text)
        l = String2Array(text)
        return l
    except AlarmException:
        # print("No new vel. Continuing...")
        return previous
    signal.signal(signal.SIGALRM, signal.SIG_IGN)

def String2Array(string):
    l = np.array(string.split(","), dtype='float')
    return l


cpdef public int nmpc_vel_ref_online() except -1:

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

    interp_nmpc = Interpolation(0.001,nmpc)

    nb_step = 10    

    f = open("../data/nmpc_vel_cython.dat", "w")
    f.write("")
    f.close()

    cdef qp = SQProblem(nmpc.nv,nmpc.nc)
    options = Options()
    options.setToMPC()
    options.printLevel = PrintLevel.LOW
    qp.setOptions(options)

    time_list = []
    i=0
    timeout = 1e-3
    nb_step_max = 125
    print("Start walking !")
    while i < nb_step_max*8:
        if i == 7:
            velocity_reference = np.array([0.2, 0., 0.])
        velocity_reference  = nonBlockingRawInput(velocity_reference,timeout)
        if 8*(nb_step_max-2)-1 <= i:
            velocity_reference = np.array([0., 0., 0.])

        if (i%8 == 0):
            print("step : ",i//8,velocity_reference)
        time_iter = i*0.1



        start_time = time.time()

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
        interp_nmpc.interpolate(time_iter)

        comx, comy, comz, footx, footy, footq, foot, comq, state = nmpc.update()
        # print(comx[0],nmpc.C_kp1_x[0],nmpc.C_kp1_x[-1],interp_nmpc.curLeft.x,interp_nmpc.CoMbuffer[-1].x)
        nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)

        time_list.append(time.time() - start_time)

        i += 1
        
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

        # print(nmpc.fsm_states,foot,foot_bool,state_bool)

        f = open("../data/nmpc_vel_cython.dat", "a")
        line = str(time.time()) + " " + str(comx[0])+ "  " + str(comx[1])+ "  " + str(comx[2])+ "  " +\
            str(comy[0])+ "  " + str(comy[1])+ "  " + str(comy[2])+ "  " +\
            str(comz)+ "  0  0  " + str(comq[0]) + "  " + str(comq[1]) + "  " +\
            str(comq[2]) + "  " + str(footx) + "  " + str(footy)+ "  " +\
            str(footq) +  "  " + str(zmpx) + "  " + str(zmpy) + "  " + str(foot_bool) \
            + "  " + str(state_bool) + " \n"
        f.write(line)
        f.close()

    interp_nmpc.save_to_file("./nmpc_interpolated_cython.csv")
    print(np.sum(time_list),np.mean(time_list))

    return 0

