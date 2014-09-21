import os
import sys
import numpy

from math import cos, sin
from time import gmtime, strftime

class FiniteStateMachine(object):
    """
    Finite state machine to implement starting and stopping maneuvers for the
    pattern generator.
    """
    def __init__(self, sl=1):
        """
        initialize finite state machine

        Parameters
        ----------

        sl: int
            number of steps in inplace stepping until stop
        """

        self._states = ('D', 'L/R', 'R/L', 'Lbar/Rbar', 'Rbar/Lbar')
        self.state = 'D'
        self.sl = sl

    def transition(self, transition):
        err_str = 'transition {} not in FSM states ({})'.format(transition, self._states)
        assert transition in self._states, err_str

        if self.state == 'D':
            pass


class BaseTypeSupportFoot(object):
    """
    """

    def __init__(self, x=0, y=0, theta=0, foot="left"):
        self.x = x
        self.y = y
        self.theta = theta
        self.foot = foot
        self.ds = 0
        self.stepNumber = 0
        self.timeLimit = 0

    def __eq__(self, other):
        """ equality operator to check if A == B """
        return (isinstance(other, self.__class__)
            or self.__dict__ == other.__dict__)

    def __ne__(self, other):
        return not self.__eq__(other)


class BaseTypeFoot(object):
    """
    """
    def __init__(self, x=0, y=0, theta=0, foot="left", supportFoot=0):
        self.x = x
        self.y = y
        self.z = 0
        self.theta = theta

        self.dx = 0
        self.dy = 0
        self.dz = 0
        self.dtheta = 0

        self.ddx = 0
        self.ddy = 0
        self.ddz = 0
        self.ddtheta = 0

        self.supportFoot = supportFoot

    def __eq__(self, other):
        """ equality operator to check if A == B """
        return (isinstance(other, self.__class__)
            or self.__dict__ == other.__dict__)

    def __ne__(self, other):
        return not self.__eq__(other)


class CoMState(object):

    def __init__(self, x=0, y=0, theta=0, h_com=0.814):
        self.x = numpy.zeros( (3,) , dtype=float )
        self.y = numpy.zeros( (3,) , dtype=float )
        self.z = h_com
        self.theta = numpy.zeros( (3,) , dtype=float )


class ZMPState(object):
    """
    """
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __eq__(self, other):
        """ equality operator to check if A == B """
        return (isinstance(other, self.__class__)
            or self.__dict__ == other.__dict__)

    def __ne__(self, other):
        return not self.__eq__(other)

