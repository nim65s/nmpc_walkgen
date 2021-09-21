import numpy as np
cimport numpy as np
from wpg.helper cimport BaseTypeSupportFoot

cdef class BaseGenerator:
    cdef int N 
    cdef float T
    cdef float T_window
    cdef float T_step    
    cdef int nf
    cdef float time
       
    # cdef tuple _fsm_states

    cdef str err_str
    cdef str fsm_state 
    cdef np.ndarray fsm_states 

    cdef float a
    cdef float b
    cdef float c
    cdef float d

    cdef np.ndarray c_k_x
    cdef np.ndarray c_k_y
    cdef np.ndarray c_k_q

    cdef float h_com
    cdef float g

    cdef np.ndarray C_kp1_x
    cdef np.ndarray dC_kp1_x
    cdef np.ndarray ddC_kp1_x

    cdef np.ndarray C_kp1_y
    cdef np.ndarray dC_kp1_y
    cdef np.ndarray ddC_kp1_y

    cdef np.ndarray C_kp1_q
    cdef np.ndarray dC_kp1_q
    cdef np.ndarray ddC_kp1_q

    cdef np.ndarray dddC_k_x
    cdef np.ndarray dddC_k_y
    cdef np.ndarray dddC_k_q

    cdef np.ndarray dC_kp1_x_ref
    cdef np.ndarray dC_kp1_y_ref
    cdef np.ndarray dC_kp1_q_ref
    cdef np.ndarray local_vel_ref

    cdef float f_k_x
    cdef float f_k_y
    cdef float f_k_q

    cdef np.ndarray F_k_x
    cdef np.ndarray F_k_y
    cdef np.ndarray F_k_q 

    cdef np.ndarray F_kp1_q 

    cdef np.ndarray f_k_qL
    cdef np.ndarray f_k_qR

    cdef np.ndarray F_k_qL
    cdef np.ndarray F_k_qR

    cdef np.ndarray F_kp1_qL
    cdef np.ndarray F_kp1_qR

    cdef np.ndarray dF_k_qL
    cdef np.ndarray dF_k_qR

    cdef np.ndarray dF_kp1_qL
    cdef np.ndarray dF_kp1_qR    

    cdef np.ndarray ddF_k_qL
    cdef np.ndarray ddF_k_qR

    cdef np.ndarray ddF_kp1_qL
    cdef np.ndarray ddF_kp1_qR     

    cdef np.ndarray dddF_k_qL
    cdef np.ndarray dddF_k_qR

    cdef np.ndarray E_F
    cdef np.ndarray E_FR
    cdef np.ndarray E_FL

    cdef np.ndarray E_F_bar
    cdef np.ndarray E_FR_bar 
    cdef np.ndarray E_FL_bar 

    cdef float z_k_x 
    cdef float z_k_y

    cdef np.ndarray Z_kp1_x
    cdef np.ndarray Z_kp1_y

    cdef float omega
    cdef float xi_k_x
    cdef float xi_k_y

    cdef np.ndarray Xi_kp1_x
    cdef np.ndarray Xi_kp1_y

    cdef np.ndarray Pps
    cdef np.ndarray Ppu

    cdef np.ndarray Pvs
    cdef np.ndarray Pvu

    cdef np.ndarray Pas
    cdef np.ndarray Pau

    cdef np.ndarray Pzs
    cdef np.ndarray Pzu

    cdef np.ndarray Pxis
    cdef np.ndarray Pxiu

    cdef int nFootPosHullEdges
    cdef np.ndarray rfposhull
    cdef np.ndarray lfposhull   

    cdef np.ndarray A0r
    cdef np.ndarray ubB0r
    cdef np.ndarray A0l  
    cdef np.ndarray ubB0l

    cdef int nc_fchange_eq
    cdef np.ndarray eqAfoot    
    cdef np.ndarray eqBfoot

    cdef int nc_foot_position
    cdef np.ndarray Afoot
    cdef np.ndarray lbBfoot
    cdef np.ndarray ubBfoot

    cdef float SecurityMarginX
    cdef float SecurityMarginY

    cdef int nFootEdge
    cdef float footWidth
    cdef float footHeight
    cdef float footDistance

    cdef np.ndarray lfoot
    cdef np.ndarray rfoot
    cdef np.ndarray lfcophull
    cdef np.ndarray rfcophull
    cdef np.ndarray dscophull

    cdef np.ndarray A0rf
    cdef np.ndarray ubB0rf

    cdef np.ndarray A0lf
    cdef np.ndarray ubB0lf

    cdef np.ndarray A0drf
    cdef np.ndarray ubB0drf
    cdef np.ndarray A0dlf
    cdef np.ndarray ubB0dlf

    cdef np.ndarray PzuV
    cdef np.ndarray PzuVx
    cdef np.ndarray PzuVy

    cdef np.ndarray PzsC
    cdef np.ndarray PzsCx
    cdef np.ndarray PzsCy

    cdef np.ndarray v_kp1fc
    cdef np.ndarray v_kp1fc_x
    cdef np.ndarray v_kp1fc_y

    cdef np.ndarray PxiuV
    cdef np.ndarray PxiuVx
    cdef np.ndarray PxiuVy

    cdef np.ndarray PxisC
    cdef np.ndarray PxisCx
    cdef np.ndarray PxisCy  

    cdef np.ndarray D_kp1
    cdef np.ndarray D_kp1x
    cdef np.ndarray D_kp1y
    cdef np.ndarray b_kp1

    cdef int nc_cop
    cdef np.ndarray Acop
    cdef np.ndarray lbBcop
    cdef np.ndarray ubBcop

    cdef int nc_dcm
    cdef np.ndarray Adcm
    cdef np.ndarray lbBdcm
    cdef np.ndarray ubBdcm

    cdef int nc_fvel_eq
    cdef np.ndarray A_fvel_eq
    cdef np.ndarray B_fvel_eq

    cdef int nc_fpos_ineq
    cdef np.ndarray A_fpos_ineq
    cdef np.ndarray ubB_fpos_ineq
    cdef np.ndarray lbB_fpos_ineq

    cdef int nc_fvel_ineq
    cdef np.ndarray A_fvel_ineq
    cdef np.ndarray ubB_fvel_ineq
    cdef np.ndarray lbB_fvel_ineq

    cdef BaseTypeSupportFoot currentSupport  
    cdef np.ndarray supportDeque

    cdef np.ndarray v_kp1 
    cdef np.ndarray V_kp1

    cdef void _update_hulls(self)

    cdef void _initialize_constant_matrices(self)    
    cdef void _initialize_cop_matrices(self)
    cdef void _initialize_cp_matrices(self)
    cdef void _initialize_selection_matrix(self)
    cdef void _initialize_convex_hull_systems(self)

    cdef void ComputeLinearSystem(self,np.ndarray hull,str foot,np.ndarray A0,np.ndarray B0)

    cdef void _calculate_support_order(self)

    cdef void _update_foot_selection_matrices(self)
    cdef void _update_cop_constraint_transformation(self)
    cdef void _update_selection_matrices(self)

    cpdef void set_security_margin(self,float margin_x,float margin_y)
    cpdef void set_velocity_reference(self,np.ndarray vel_ref)
    cpdef void set_initial_values(self,np.ndarray com_x,np.ndarray com_y,float com_z,\
    float foot_x,float foot_y,float foot_q,str foot,np.ndarray com_q)

    cdef void buildConstraints(self)
    cdef void buildCoPconstraint(self)   
    cdef void buildFootEqConstraint(self)
    cdef void buildFootIneqConstraint(self)
    cdef void buildFootRotationConstraints(self)
    cdef void buildRotIneqConstraint(self)
       
    cpdef void simulate(self)
    cpdef tuple update(self)