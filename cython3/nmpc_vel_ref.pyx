from wpg.combinedqp cimport NMPCGenerator
import time
import numpy as np
cimport numpy as np
import os, sys
from cpython cimport array

cpdef public int nmpc_vel_ref() except -1:

    cdef np.ndarray comx
    # comy
    # comz,comq,footx,footy,footq
    cdef np.ndarray velocity_reference
    cdef int nb_step
    cdef str foot = 'left'
    cdef str state = 'D'

    nmpc = NMPCGenerator(fsm_state=state)
    nmpc.set_security_margin(0.09,0.05)

    velocity_reference = np.array([0.,0.,0.])

    comx = np.array([-3.16e-3, 0.0, 0.0])
    comy = np.array([1.237384291203724555e-03,0.0, 0.0])
    comz = 8.786810585901939641e-01
    comq = 0.0
    footx = 1.86e-4
    footy = 0.085
    footq = 0.0 

    nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)

    nb_step = 10    

    f = open("../data/nmpc_vel_cython.dat", "w")
    f.write("")
    f.close()

    for i in range(8*nb_step):
        print("iteration : ",i)

        if 7 <= i < 8*(nb_step-2)-1 :
            velocity_reference = np.array([0.2, 0., 0.])
        if 8*(nb_step-2)-1 <= i:
            velocity_reference = np.array([0., 0., 0.])

        nmpc.set_velocity_reference(velocity_reference)

        nmpc.solve()
        nmpc.simulate()

        # comx, comy, comz, footx, footy, footq, foot, comq, state = nmpc.update()
        comx,comq = nmpc.update()
        print(comx,comq)
        nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)

        f = open("../data/nmpc_vel_cython.dat", "a")
        line = str(time.time()) + " " + str(comx[0]) + "\n"
        f.write(line)
        f.close()

    return 0

