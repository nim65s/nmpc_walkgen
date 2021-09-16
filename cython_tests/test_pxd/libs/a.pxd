import numpy as np
cimport numpy as np

DTYPE = np.int
ctypedef np.int_t DTYPE_t

cpdef int myfunction(int x, int y=*)

cdef double _helper(double a)

cdef class A:
    cdef public int a, b
    cdef np.ndarray l 
    cpdef np.ndarray foo(self, double x)
