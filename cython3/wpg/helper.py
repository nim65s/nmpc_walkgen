import numpy as np

class BaseTypeSupportFoot(object):

    def __init__(self, x=0, y=0, theta=0, foot='left'):
        self.x = x
        self.y = y
        self.q = theta
        self._foot = foot
        self._ds = 0
        self._stepNumber = 0
        self._timeLimit = 0

    def get_ds(self):
        return self._ds

    def set_ds(self, ds):
        self._ds = ds

    ds = property(get_ds, set_ds)

    def get_foot(self):
        return self._foot

    def set_foot(self, foot):
        self._foot = foot

    foot = property(get_foot, set_foot)

    def get_stepNumber(self):
        return self._stepNumber

    def set_stepNumber(self, stepNumber):
        self._stepNumber = stepNumber

    stepNumber = property(get_stepNumber, set_stepNumber) 

    def get_timeLimit(self):
        return self._timeLimit

    def set_timeLimit(self, timeLimit):
        self._timeLimit = timeLimit

    timeLimit = property(get_timeLimit, set_timeLimit)

class CoMState(object):

    def __init__(self, h_com=0.814):
        self._x = np.zeros((3,),dtype=float)
        self._y = np.zeros((3,),dtype=float)
        self._z = h_com
        self._q = np.zeros((3,),dtype=float)

    def get_x(self):
        return self._x

    def set_x(self, x):
        self._x = x

    x = property(get_x, set_x)

    def get_y(self):
        return self._y

    def set_y(self, y):
        self._y = y

    y = property(get_y, set_y)

    def get_z(self):
        return self._z

    def set_z(self, z):
        self._z = z

    z = property(get_z, set_z)

    def get_q(self):
        return self._q

    def set_q(self, q):
        self._q = q

    q = property(get_q, set_q)


class ZMPState(object):

    def __init__(self, x=0, y=0, z=0):
        self._x = x
        self._y = y
        self._z = z

    def get_x(self):
        return self._x

    def set_x(self, x):
        self._x = x

    x = property(get_x, set_x)

    def get_y(self):
        return self._y

    def set_y(self, y):
        self._y = y

    y = property(get_y, set_y)

    def get_z(self):
        return self._z

    def set_z(self, z):
        self._z = z

    z = property(get_z, set_z)

class BaseTypeFoot(object):

    def __init__(self, x=0, y=0, theta=0, foot="left", supportFoot=0):
        self._x = x
        self._y = y
        self._z = 0
        self._q = theta

        self._dx = 0
        self._dy = 0
        self._dz = 0
        self._dq = 0

        self._ddx = 0
        self._ddy = 0
        self._ddz = 0
        self._ddq = 0

        self._supportFoot = supportFoot

    def get_x(self):
        return self._x

    def set_x(self, x):
        self._x = x

    x = property(get_x, set_x)

    def get_y(self):
        return self._y

    def set_y(self, y):
        self._y = y

    y = property(get_y, set_y)

    def get_z(self):
        return self._z

    def set_z(self, z):
        self._z = z

    z = property(get_z, set_z)

    def get_q(self):
        return self._q

    def set_q(self, q):
        self._q = q

    q = property(get_q, set_q)        

    def get_dx(self):
        return self._dx

    def set_dx(self, dx):
        self._dx = dx

    dx = property(get_dx, set_dx)

    def get_dy(self):
        return self._dy

    def set_dy(self, dy):
        self._dy = dy

    dy = property(get_dy, set_dy)

    def get_dz(self):
        return self._dz

    def set_dz(self, dz):
        self._dz = dz

    dz = property(get_dz, set_dz)

    def get_dq(self):
        return self._dq

    def set_dq(self, dq):
        self._dq = dq

    dq = property(get_dq, set_dq)

    def get_ddx(self):
        return self._ddx

    def set_ddx(self, ddx):
        self._ddx = ddx

    ddx = property(get_ddx, set_ddx)

    def get_ddy(self):
        return self._ddy

    def set_ddy(self, ddy):
        self._ddy = ddy

    ddy = property(get_ddy, set_ddy)

    def get_ddz(self):
        return self._ddz

    def set_ddz(self, ddz):
        self._ddz = ddz

    ddz = property(get_ddz, set_ddz)

    def get_ddq(self):
        return self._ddq

    def set_ddq(self, ddq):
        self._ddq = ddq

    ddq = property(get_ddq, set_ddq)

    def get_supportFoot(self):
        return self._supportFoot

    def set_supportFoot(self, supportFoot):
        self._supportFoot = supportFoot

    supportFoot = property(get_supportFoot, set_supportFoot)