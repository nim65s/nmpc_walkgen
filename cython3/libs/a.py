import numpy as np
from qpoases import PySQProblem as SQProblem
from libs import c

def myfunction(x, y=2):
    l = np.array([x,y],dtype=int)
    return l[0]+l[1]


def _helper(a):
    return a + 1


class A:
    def __init__(self, b=0):
        self.a = 3
        self.b = b
        self.l = np.zeros((2,2))
        self.nv = 10
        self.nc = 10        
        
        self.qp_H   =  np.eye(self.nv,self.nv)
        self.qp_A   =  np.zeros((self.nc,self.nv))
        self.qp_g   =  np.zeros((self.nv,))
        self.qp_lb  = -np.ones((self.nv,))*1e+08
        self.qp_ub  =  np.ones((self.nv,))*1e+08
        self.qp_lbA = -np.ones((self.nc,))*1e+08
        self.qp_ubA =  np.ones((self.nc,))*1e+08        
        self.cpu_time = 2.9 # ms
        self.nwsr = 1000 # unlimited bounded
        self.txt = "left"

    def foo(self, x):
        print(x + _helper(1.0))
        return self.l

    def bar(self):
        test = c.C()
        return test.c(3)  

    def test_char(self):
        print(self.txt == "left")
        return 0

    def test_qp(self):
        qp = SQProblem(self.nv, self.nc)
        qp.init(
            self.qp_H, self.qp_g, self.qp_A,
            self.qp_lb, self.qp_ub,
            self.qp_lbA, self.qp_ubA,
            self.nwsr, self.cpu_time
        )
        return(0)

