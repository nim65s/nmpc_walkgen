import numpy as np
cimport numpy as np

cdef class BaseTypeSupportFoot:

    cdef float x
    cdef float y
    cdef float q
    cdef str _foot
    cdef int _ds
    cdef int _stepNumber
    cdef float _timeLimit

cdef class CoMState:

    cdef np.ndarray _x,_y,_q
    cdef float _z

cdef class ZMPState:

    cdef float _x,_y,_z

cdef class BaseTypeFoot:

    cdef float _x,_y,_z,_q,_dx,_dy,_dz,_dq,_ddx,_ddy,_ddz,_ddq
    cdef int _supportFoot