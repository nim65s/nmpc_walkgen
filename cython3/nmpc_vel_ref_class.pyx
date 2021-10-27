from wpg.combinedqp cimport NMPCGenerator
from wpg.interpolation cimport Interpolation
import time
import numpy as np
cimport numpy as np
import os, sys
from libcpp.vector cimport vector
from libcpp.string cimport string
# cdef public class Nmpc[object Nmpc, type NmpcType]:
#     cdef np.ndarray comx, comy, comq, vel_ref
#     cdef float comz, footx, footy, footq
#     cdef str foot, state
#     cdef nmpc
#     cdef interp_nmpc

#     def __cinit__(self,np.ndarray comx,np.ndarray comy,float comz,float footx,\
#             float footy,float footq,str foot,np.ndarray comq,str state,\
#             np.ndarray vel_ref):
#         self.comx = comx
#         self.comy = comy
#         self.comz = comz   
#         self.comq = comq  

#         self.footx = footx
#         self.footy = footy
#         self.footq = footq
#         self.foot = foot 

#         self.state = state

#         self.velocity_reference = vel_ref 

#         self.nmpc = NMPCGenerator(fsm_state=self.state)
#         self.nmpc.set_security_margin(0.09,0.05)
#         self.nmpc.set_initial_values(self.comx, self.comy, self.comz, self.footx,\
#             self.footy, self.footq, self.foot, self.comq)

#         self.interp_nmpc = Interpolation(0.001,self.nmpc)

    # cdef void set_vel_ref(self,np.ndarray vel_ref):
    #     self.velocity_reference = vel_ref 

    # cdef void solve(self,double time_iter):
    #     self.nmpc.set_velocity_reference(self.velocity_reference)
    #     self.nmpc.solve()
    #     self.nmpc.simulate()
    #     self.interp_nmpc.interpolate(time_iter)

    #     self.comx, self.comy, self.comz, self.footx, self.footy, self.footq,\
    #         self.foot, self.comq, self.state = self.nmpc.update()
    #     self.nmpc.set_initial_values(self.comx, self.comy, self.comz, self.footx,\
    #         self.footy, self.footq, self.foot, self.comq)

    # cdef void export_interp(self,str path):
    #     self.interp_nmpc.save_to_file(path)

# cdef api Nmpc buildNmpc(float comx,float dcomx,float ddcomx,float comy,\
#         float dcomy,float ddcomy,float comz,float footx,float footy,\
#         float footq,str foot,float comq,float dcomq,float ddcomq,str state,\
#         float vx, float vy,float vq):
#     comx = np.array([comx,dcomx,ddcomx])
#     comy = np.array([comy,dcomy,ddcomy])    
#     comq = np.array([comq,dcomq,ddcomq])
#     vel_ref = np.array([vx,vy,vq])     
#     return Nmpc(comx, comy, comz, footx, footy, footq, foot, comq, state, vel_ref)



cdef public class Nmpc[object Nmpc, type NmpcType]:
    cdef string a
    cdef nmpc
    cdef interp_nmpc

    def __cinit__(self,string a):
        self.a = a
        self.nmpc = NMPCGenerator(fsm_state='D')
        self.nmpc.set_security_margin(0.09,0.05)
        self.interp_nmpc = Interpolation(0.001,self.nmpc)

    cdef double solve(self,double i):
        return i


cdef api Nmpc buildNmpc(string a):
    print("here")
    return(Nmpc(a))

cdef api double solveNmpc(Nmpc n,double d):
    print("there")
    print(n.a,n.solve(d))
    return(n.solve(d))

# cpdef public int nmpc_vel_ref() except -1:

#     cdef np.ndarray comx, comy, comq, velocity_reference
#     cdef float comz, footx, footy, footq
#     cdef int nb_step
#     cdef str foot = 'left'
#     cdef str state = 'D'

#     nmpc = NMPCGenerator(fsm_state=state)

    # nmpc.set_security_margin(0.09,0.05)

    # velocity_reference = np.array([0.,0.,0.])

    # comx = np.array([-3.16e-3, 0.0, 0.0])
    # comy = np.array([1.237384291203724555e-03,0.0, 0.0])
    # comz = 8.786810585901939641e-01
    # comq = np.array([0.,0.,0.])
    # footx = 1.86e-4
    # footy = 0.085
    # footq = 0.0 

    # nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)

    # interp_nmpc = Interpolation(0.001,nmpc)

    # nb_step = 10    

    # f = open("../data/nmpc_vel_cython.dat", "w")
    # f.write("")
    # f.close()

    # time_list = []
    # for i in range(8*nb_step):
    #     # print("iteration : ",i)
    #     time_iter = i*0.1

    #     # if 7 <= i < 8*4-1 :
    #     #     velocity_reference = np.array([0.2, 0.0, 0.0])
    #     # if 8*4-1 <= i < 8*15-1 :
    #     #     velocity_reference = np.array([0.1, 0., 0.2])
    #     # if 8*15-1 <= i < 8*22-1 :
    #     #     velocity_reference = np.array([0.2, 0.0, 0.1])
    #     # if 8*22-1 <= i < 8*(nb_step-2)-1 :
    #     #     velocity_reference = np.array([0.2, 0.0, -0.1])
    #     if 7 <= i < 8*(nb_step-2)-1 :
    #         velocity_reference = np.array([0.2, 0., 0.])
    #     if 8*(nb_step-2)-1 <= i:
    #         velocity_reference = np.array([0., 0., 0.])

    #     start_time = time.time()

    #     nmpc.set_velocity_reference(velocity_reference)

    #     nmpc.solve()
    #     nmpc.simulate()
    #     interp_nmpc.interpolate(time_iter)

    #     comx, comy, comz, footx, footy, footq, foot, comq, state = nmpc.update()
    #     # print(comx[0],nmpc.C_kp1_x[0],nmpc.C_kp1_x[-1],interp_nmpc.curLeft.x,interp_nmpc.CoMbuffer[-1].x)
    #     nmpc.set_initial_values(comx, comy, comz, footx, footy, footq, foot, comq)

    #     time_list.append(time.time() - start_time)
        
    #     zmpx = comx[0]-comz/9.81*comx[2]
    #     zmpy = comy[0]-comz/9.81*comy[2]  

    #     if state == 'D':
    #         state_bool = 0
    #     elif state == 'L':
    #         state_bool = 1
    #     else:
    #         state_bool = -1
    #     if foot == 'left':
    #         foot_bool = 1
    #     else :
    #         foot_bool = -1

    #     # print(nmpc.fsm_states,foot,foot_bool,state_bool)

    #     f = open("../data/nmpc_vel_cython.dat", "a")
    #     line = str(time.time()) + " " + str(comx[0])+ "  " + str(comx[1])+ "  " + str(comx[2])+ "  " +\
    #         str(comy[0])+ "  " + str(comy[1])+ "  " + str(comy[2])+ "  " +\
    #         str(comz)+ "  0  0  " + str(comq[0]) + "  " + str(comq[1]) + "  " +\
    #         str(comq[2]) + "  " + str(footx) + "  " + str(footy)+ "  " +\
    #         str(footq) +  "  " + str(zmpx) + "  " + str(zmpy) + "  " + str(foot_bool) \
    #         + "  " + str(state_bool) + " \n"
    #     f.write(line)
    #     f.close()

    # interp_nmpc.save_to_file("./nmpc_interpolated_cython.csv")
    # print(np.sum(time_list),np.mean(time_list))

    # return 0

