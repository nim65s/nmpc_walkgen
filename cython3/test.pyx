from libs cimport a
from libcpp.vector cimport vector
import time
import numpy as np
import os, sys
from qpoases import PySQProblem as SQProblem
import signal

class AlarmException(Exception):
    pass

def alarmHandler(signum, frame):
    raise AlarmException

def nonBlockingRawInput(previous,timeout):
    signal.signal(signal.SIGALRM, alarmHandler)
    signal.setitimer(signal.ITIMER_REAL,timeout,0.0)
    try:
        text = raw_input('new vel ? \n')
        signal.alarm(0)
        print("new vel !", text)
        l = String2Array(text)
        return l
    except AlarmException:
        print('No new vel. Continuing...')
        return previous
    signal.signal(signal.SIGALRM, signal.SIG_IGN)

def String2Array(string):
    l = np.array(string.split(","), dtype='float')
    return l

# cpdef public vector[int] test(int x):
cpdef public int test(int x):
    print("x =",x)

    cdef qp = SQProblem(2, 1)

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

    # w = int(input("enter int w :"))
    # print(w+2)
    i = 0
    timeout = 1e-3
    l = np.zeros((1,3))
    while True:
    # for i in range(8*nb_step):
        print("iteration : ",i,l,type(l))

        l = nonBlockingRawInput(l,timeout)

        f = open("../data/test.dat", "a")
        line = str(time.time()) +" \n"
        f.write(line)
        f.close()

        i+= 1


    print(nmpc.bar())

    nmpc.test_qp()

    # cdef vector[int] m
    # m = [1,2,3]

    return 0

