import numpy as np
cimport numpy as np

# from libcpp.qpoases import SQProblem 
# from qpoases import PySQProblem as SQProblem


# DTYPE = np.int
# ctypedef np.int_t DTYPE_t

cpdef int myfunction(int x, int y=*)

cdef double _helper(double a)

cdef class A:
    cdef public int a, b
    cdef np.ndarray l 
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

    cpdef np.ndarray foo(self, double x)

    cpdef int test_qp(self)

