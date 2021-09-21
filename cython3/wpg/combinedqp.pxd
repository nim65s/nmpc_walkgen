import numpy as np
cimport numpy as np
from cpython cimport array
from wpg.base cimport BaseGenerator

cdef class NMPCGenerator(BaseGenerator):
    
    cdef np.ndarray l


    cpdef int solve(self)   
