import numpy as np
cimport numpy as np
from cpython cimport array
from wpg.base cimport BaseGenerator


cdef class NMPCGenerator(BaseGenerator):
    
    cdef int nv
    cdef int nc_pos
    cdef int nc_ori
    cdef int nc

    cdef np.ndarray nwsr,cputime,qp_nwsr,qp_cputime
    cdef options

    cdef np.ndarray dofs
    cdef qp

    cpdef np.ndarray qp_H     
    cpdef np.ndarray qp_A
    cpdef np.ndarray qp_g
    cpdef np.ndarray qp_lb    
    cpdef np.ndarray qp_ub
    cpdef np.ndarray qp_lbA
    cpdef np.ndarray qp_ubA

    cdef bint _qp_is_initialized # or bool + from libcpp cimport bool

    cdef np.ndarray Hx
    cdef np.ndarray Q_k_x    
    cdef np.ndarray p_k_x
    cdef np.ndarray p_k_y

    cdef np.ndarray Hq
    cdef np.ndarray Q_k_qR    
    cdef np.ndarray Q_k_qL
    cdef np.ndarray p_k_qR
    cdef np.ndarray p_k_qL

    cdef np.ndarray A_pos_x
    cdef np.ndarray A_pos_q    
    cdef np.ndarray ubA_pos
    cdef np.ndarray lbA_pos

    cdef np.ndarray A_ori
    cdef np.ndarray ubA_ori    
    cdef np.ndarray lbA_ori

    cdef np.ndarray derv_Acop_map   
    cdef np.ndarray derv_Afoot_map

    cdef void _calculate_common_expressions(self)
    cdef void _calculate_derivatives(self)

    cpdef void solve(self)
    cdef void _preprocess_solution(self)
    cdef void _solve_qp(self)
    cdef void _postprocess_solution(self)       

    cdef void _update_foot_selection_matrix(self) 
    
    cpdef tuple update(self)