import numpy as np
cimport numpy as np
from cpython cimport array
from wpg.base cimport BaseGenerator

cdef class NMPCGenerator(BaseGenerator):
    
    cdef np.ndarray l

    cpdef void print_fct(self)

    # cpdef int set_security_margin(self,double a,double b)

    cpdef int set_initial_values(self,np.ndarray comx,np.ndarray  comy,float  comz,\
    float footx,float footy,float footq,str foot,float comq)

    cpdef int set_velocity_reference(self,np.ndarray vel_ref)

    cpdef int solve(self)   

    cpdef int simulate(self)

    cpdef tuple update(self)