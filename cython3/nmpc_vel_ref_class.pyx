from wpg.combinedqp cimport NMPCGenerator
from wpg.interpolation cimport Interpolation
import time
import numpy as np
cimport numpy as np
import os, sys


cdef public class Nmpc[object Nmpc, type NmpcType]:
    cdef np.ndarray comx, comy, comq, velocity_reference
    cdef float comz, footx, footy, footq
    cdef str foot, state
    cdef NMPCGenerator nmpc
    cdef Interpolation interp_nmpc

    def __cinit__(self,np.ndarray comx,np.ndarray comy,float comz,float footx,\
            float footy,float footq,str foot,np.ndarray comq,str state,\
            np.ndarray vel_ref):
        self.comx = comx
        self.comy = comy
        self.comz = comz   
        self.comq = comq  

        self.footx = footx
        self.footy = footy
        self.footq = footq
        self.foot = foot 

        self.state = state

        self.velocity_reference = vel_ref 

        self.nmpc = NMPCGenerator(fsm_state=self.state)
        self.nmpc.set_security_margin(0.09,0.05)
        self.nmpc.set_initial_values(self.comx, self.comy, self.comz, self.footx,\
            self.footy, self.footq, self.foot, self.comq)

        self.interp_nmpc = Interpolation(0.001,self.nmpc)
        # self.interp_nmpc.save_to_file("./test.csv")

    cdef void set_vel_ref(self,np.ndarray vel_ref):
        self.velocity_reference = vel_ref 

    cdef void solve(self,int i):
        time_iter = i*self.nmpc.T
        self.nmpc.set_velocity_reference(self.velocity_reference)
        self.nmpc.solve()
        self.nmpc.simulate()
        self.interp_nmpc.interpolate(time_iter)

        self.comx, self.comy, self.comz, self.footx, self.footy, self.footq,\
            self.foot, self.comq, self.state = self.nmpc.update()
        self.nmpc.set_initial_values(self.comx, self.comy, self.comz, self.footx,\
            self.footy, self.footq, self.foot, self.comq)

    cdef void export_interp(self,str path):
        self.interp_nmpc.save_to_file(path)

cdef api Nmpc buildNmpc(float comx,float dcomx,float ddcomx,float comy,\
        float dcomy,float ddcomy,float comz,float footx,float footy,\
        float footq,float comq,float dcomq,float ddcomq,\
        float vx, float vy,float vq,const char* ft,const char* st):
    com_x = np.array([comx,dcomx,ddcomx])
    com_y = np.array([comy,dcomy,ddcomy])    
    com_q = np.array([comq,dcomq,ddcomq])
    vel_ref = np.array([vx,vy,vq])   
    foot = ft.decode('UTF-8')
    state = st.decode('UTF-8')
    return Nmpc(com_x, com_y, comz, footx, footy, footq, foot, com_q, state, vel_ref)

cdef api void set_velocity_referenceNmpc(Nmpc nmpc,float vx,float vy,float vq):
    new_vel = np.array([vx,vy,vq])
    nmpc.set_vel_ref(new_vel)

cdef api void read_velocity_referenceNmpc(Nmpc nmpc):
    print("new vel ref :",nmpc.velocity_reference)

cdef api void solveNmpc(Nmpc nmpc,int i):
    nmpc.solve(i)

cdef api void interpolationNmpc(Nmpc nmpc,const char* path):
    nmpc.export_interp(path.decode('UTF-8'))
