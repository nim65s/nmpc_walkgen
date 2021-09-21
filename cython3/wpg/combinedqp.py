import numpy as np
from qpoases import PySQProblem as SQProblem
from wpg.base import BaseGenerator

class NMPCGenerator(BaseGenerator):

    def __init__(self, N=16, T=0.2, T_step=1.6, fsm_state='D', fsm_sl=1):
        super(NMPCGenerator, self).__init__(N, T, T_step, fsm_state, fsm_sl)
        self.l = np.array([0,0,0])

    def solve(self):
        return 0

