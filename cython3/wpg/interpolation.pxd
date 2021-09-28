import numpy as np
cimport numpy as np
from wpg.base cimport BaseGenerator
from wpg.helper cimport CoMState, ZMPState, BaseTypeFoot, BaseTypeSupportFoot

cdef class Interpolation:    

	cdef BaseGenerator gen

	cdef double T
	cdef double Tc
	cdef int interval
	# cdef str state

	cdef CoMState curCoM
	cdef ZMPState zmp

	cdef BaseTypeFoot curLeft
	cdef BaseTypeFoot curRight

	cdef np.ndarray CoMbuffer
	cdef np.ndarray ZMPbuffer
	cdef np.ndarray RFbuffer
	cdef np.ndarray LFbuffer

	cdef np.ndarray comTraj
	cdef np.ndarray zmpTraj
	cdef np.ndarray leftFootTraj
	cdef np.ndarray rightFootTraj

	cdef LIPM lipm
	cdef FootInterpolation fi	

	cdef void interpolate(self,double time)	
	cdef void save_to_file(self,str filename)

cdef class LIPM:

	cdef double Tc
	cdef double T
	cdef float h_com
	cdef float g

	cdef np.ndarray A
	cdef np.ndarray B		
	cdef np.ndarray C
	cdef np.ndarray Ac
	cdef np.ndarray Bc	

	cdef int intervaleSize

	cdef void initializeSystem(self)
	cdef tuple interpolate(self,float jerkX,float jerkY,CoMState curCoM,
		np.ndarray ZMPbuffer,np.ndarray CoMbuffer)


cdef class FootInterpolation:

	cdef double T
	cdef double Tc
	cdef double feetDist
	cdef double stepHeigth

	cdef Polynome5 polynomeX
	cdef Polynome5 polynomeY
	cdef Polynome5 polynomeQ
	cdef Polynome4 polynomeZ

	cdef double TSS
	cdef double TDS
	cdef double stepTime
	cdef int intervaleSize
	cdef BaseGenerator gen

	cdef tuple interpolate(self,double time,BaseTypeSupportFoot currentSupport,BaseTypeFoot curLeft,
		BaseTypeFoot curRight,float F_k_x,float F_k_y,
		float PreviewAngle,np.ndarray LeftFootBuffer,
		np.ndarray RightFootBuffer)
	cdef BaseTypeFoot computeXYQ(self,BaseTypeFoot foot,double t)

cdef class Polynome:
	
	cdef int degree
	cdef np.ndarray coef

	cdef double compute(self,double time)
	cdef double computeDerivative(self,double time)
	cdef double computeSecDerivative(self,double time)

cdef class Polynome5(Polynome):

	cdef double _FT
	cdef double FP
	cdef double IP
	cdef double IS
	cdef double IA

	cdef void setParameters(self,double FinalTime,double FinalPosition,double InitialPosition,double InitialSpeed,double InitialAcceleration)

cdef class Polynome4(Polynome):

	cdef double _FT
	cdef double MP

	cdef void setParameters(self,double FinalTime,double MiddlePosition,double InitialPosition,double InitialSpeed)