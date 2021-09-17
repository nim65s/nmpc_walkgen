from libs cimport a
# from libcpp.string cimport string
import time
import numpy as np
import os, sys

cpdef public int test() except -1:

    nmpc = a.A()
    f = open("../data/test.dat", "w")
    f.write("")
    f.close()

    cdef float comx[3],velocity_reference[3]
    cdef int nb_step
    cdef str foot = 'left'

    velocity_reference = [0., 0.,0.0]
    comx = [-3.16e-3, 0.0, 0.0]
    nb_step = 10
    print(comx,velocity_reference,nb_step)

    print(nmpc.test_char())
    print(foot)

    for i in range(8*nb_step):
        # print("iteration : ",i)
        # print(nmpc.foo(i))

        f = open("../data/test.dat", "a")
        line = str(time.time()) +" \n"
        f.write(line)
        f.close()

    print(nmpc.bar())

    nmpc.test_qp()

    return 0

