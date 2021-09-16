from libs cimport a
from libcpp.vector cimport vector
import time
import numpy as np

cpdef public int nmpc_vel_ref() except -1:
    nmpc = a.A()

    cdef float comx[3]
    cdef int nb_step
    cdef float velocity_reference[3]

    velocity_reference = [0., 0.,0.0]
    comx = [-3.16e-3, 0.0, 0.0]
    nb_step = 10
    print(comx,velocity_reference,nb_step)

    for i in range(8*nb_step):
        print("iteration : ",i)
        print(nmpc.foo(i))

    return 0

# nmpc_vel_ref()
