import numpy as np
cimport numpy as np
from libs cimport c

# cimport qpoases
# from libcpp.qpoases import SQProblem 
from qpoases import PySQProblem as SQProblem
# from qpoases cimport PySQProblem as SQProblem


# cpdef int myfunction(int x, int y=*)


cdef class A:
    # cdef public int a, b
    # cdef np.ndarray l 
    cdef int nv
    cdef int nc       
    cdef np.ndarray qp_H 
    cdef np.ndarray qp_A  
    cdef np.ndarray qp_g  
    cdef np.ndarray qp_lb 
    cdef np.ndarray qp_ub  
    cdef np.ndarray qp_lbA 
    cdef np.ndarray qp_ubA       
    cdef float cpu_time 
    cdef int nwsr 
    # cdef str txt
    cdef SQProblem qp

    # cpdef int bar(self)

    # cpdef int test_char(self)

    cpdef int test_qp(self)


