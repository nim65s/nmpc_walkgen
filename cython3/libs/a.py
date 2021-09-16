import numpy as np

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

    def foo(self, x):
        print(x + _helper(1.0))
        return self.l
