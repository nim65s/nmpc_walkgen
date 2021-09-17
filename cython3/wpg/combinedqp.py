import numpy as np
from qpoases import PySQProblem as SQProblem

class NMPCGenerator:

    def __init__(self, N=16, T=0.2, T_step=1.6, fsm_state='D', fsm_sl=1):
        self.N = N
        self.l = np.array([0,0,0])

    def set_security_margin(self,a,b):
        return 0

    def set_initial_values(self, comx, comy, comz, footx, footy, footq, foot, comq):
        return 0

    def set_velocity_reference(self,vel_ref):
        return 0

    def solve(self):
        return 0

    def simulate(self):
        return 0

    def update(self):
        return (self.l,self.N)