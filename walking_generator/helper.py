import os
import sys
import numpy
import json
from math import cos, sin
from time import gmtime, strftime

import matplotlib
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec

matplotlib.rc('xtick', labelsize=6)
matplotlib.rc('ytick', labelsize=6)
matplotlib.rc('font', size=10)
matplotlib.rc('text', usetex=True)

class PlotData(object):
    """
    Smart data container for saving plotting relevant data.
    """

    def __init__(self, generator, member=()):
        """ build data structures """
        self.generator = generator
        self.data = {'time' : []}

        if not member:
            err_str = 'please provide member list, else nothing can be saved.'
            sys.stderr.write(err_str + '\n')
        else:
            for key in member:
                if not type(key) == str:
                    err_str = 'please provide only member names, {} is not a string'.format(key)
                    raise AttributeError
                self.data[key] = []

            self.member = self.data.keys()

    def update(self):
        """ update internal data from generator """
        for key in self.data.keys():
            val = self.generator.__dict__[key]
            # when numpy array convert it to list
            if isinstance(val, numpy.ndarray):
                val = val.tolist()
            self.data[key].append(val)

    def save_to_file(self, filename=None):
        """ save data in json format to file """
        # generate general filename
        if not filename:
            timestamp = strftime("%Y-%m-%d-%H-%M-%S")
            filename = '{stamp}_generator_data.json'.format(stamp=timestamp)

        with open(filename, 'w') as f:
            json.dump(self.data, f, sort_keys=True, indent=2)


class Plotter(object):
    """
    class that generates trajectories from pattern generator json data.
    """
    def __init__(self, filename):
        # try to read data
        with open(filename, 'r') as f:
            self.data = json.load(f)

        print self.data

    def _generate_consistent_data(self):
        pass


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

