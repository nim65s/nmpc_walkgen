import numpy as np
cimport numpy as np
from cpython cimport array

cdef class NMPCGenerator:
    cdef int N 
    cdef np.ndarray l

    cpdef int set_security_margin(self,double a,double b)

    cpdef int set_initial_values(self,np.ndarray comx,np.ndarray  comy,float  comz,\
    float footx,float footy,float footq,str foot,float comq)

    cpdef int set_velocity_reference(self,np.ndarray vel_ref)

    cpdef int solve(self)   

    cpdef int simulate(self)

    cpdef tuple update(self)