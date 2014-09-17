import os
import sys
import numpy
import json
from math import cos, sin
from time import gmtime, strftime

import matplotlib
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec

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
            self.data[key].append(val)

    def save_to_file(self, filename=None):
        """ save data in json format to file """
        # generate general filename
        if not filename:
            timestamp = strftime("%Y-%m-%d-%H-%M-%S")
            filename = '{stamp}_generator_data.json'.format(stamp=timestamp)

        # convert numpy arrays into lists
        for key in self.data.keys():
            val = self.generator.__dict__[key]
            if isinstance(val, numpy.ndarray):
                val = val.tolist()
            self.data[key].append(val)

        with open(filename, 'w') as f:
            json.dump(self.data, f, sort_keys=True, indent=2)


class Plotter(object):
    """
    class that generates trajectories from pattern generator json data.
    """
    def __init__(self, generator=None, show_canvas=True, save2file=False):
        # setting matplotlib to pretty printing
        matplotlib.rc('xtick', labelsize=6)
        matplotlib.rc('ytick', labelsize=6)
        matplotlib.rc('font', size=10)
        matplotlib.rc('text', usetex=True)

        # some plotting options
        self.figsize = (8.0, 5.0)
        self.dpi     = 300

        # save reference to pattern generator
        # NOTE used for online plotting
        self.generator = generator

        # decides if plots are shown in a canvas
        self.show_canvas = show_canvas

        # decides if plots are saved as pictures
        self.save2file = save2file

        # initialize plotter
        self.waterfall_fig = plt.figure()
        self.reference_fig = plt.figure()

        # use interactive mode for "real time" plotting
        if self.show_canvas:
            plt.interactive(True)

    def update_data_from_generator(self):
        """ update plots from generator internal data """
        # get data from generator
        # TODO * how to merge two dictionaries
        #      * get numpy arrays from data
        pass

    def update_data_from_file(self, filename):
        """
        load data from file and merge it into internal data structure

        Parameters
        ----------

        filename: str
            path to file that is opened via open(filename, 'r')

        .. NOTE: Expects data to be in json format!

        """
        pass

    def load_from_file(self, filename):
        """
        load data from file

        Parameters
        ----------

        filename: str
            path to file that is opened via open(filename, 'r')

        .. NOTE: Expects data to be in json format!

        """

        # check if file exists
        if not os.path.isfile(filename):
            err_str = 'filename is not a proper path to file:\n filename = {}'.format(filename)
            raise IOError(err_str)

        self.input_filename = filename

        # try to read data
        with open(self.input_filename, 'r') as f:
            self.data = json.load(f)

        # retrieve numpy arrays from data
        self._generate_consistent_data()

    def _generate_consistent_data(self):
        """ convert all lists to numpy arrays for convenience """
        for key in self.data.keys():
            val = self.data[key]
            self.data[key] = numpy.asarray(val, dtype=type(val))

    def create_reference_plot(self):
        """ create plot like that from Maximilien """
        ax = self.reference_fig.add_subplot(1,1,1)
        ax.set_ylabel("Walking Forward")
        ax.set_xlabel("time [s]")

        # assemble different trajectories
        self.reference_lines = []

        # retrieve data from data structure
        time    = self.data['time']

        # x values
        c_k_x   = self.data['c_k_x']
        f_k_x   = self.data['f_k_x']
        C_kp1_x = self.data['C_kp1_x']
        F_kp1_x = self.data['F_kp1_x']
        Z_kp1_x = self.data['Z_kp1_x']

        # y values
        c_k_y   = self.data['c_k_y']
        f_k_y   = self.data['f_k_y']
        C_kp1_y = self.data['C_kp1_y']
        F_kp1_y = self.data['F_kp1_y']
        Z_kp1_y = self.data['Z_kp1_y']

        # q values
        c_k_q   = self.data['c_k_q']
        f_k_q   = self.data['f_k_q']
        C_kp1_q = self.data['C_kp1_q']
        F_kp1_q = self.data['F_kp1_q']

        # CoM trajectory
        # x values
        line, = ax.plot([], [],
            lw='1', ls='-', marker='', ms=4, c='r',
            label='c_{k}^{x}'
        )

        x_data = time
        y_data = numpy.ones(time.shape[0])*numpy.nan
        for i in range(y_data.shape[0]):
            y_data[i] = c_k_x[i,0]

        line.set_xdata(x_data)
        line.set_ydata(y_data)

        self.reference_lines.append(line)

        # y values
        line, = ax.plot([], [],
            lw='1', ls='-.', marker='', ms=4, c='k',
            label='c_{k}^{y}'
        )

        x_data = time
        y_data = numpy.ones(time.shape[0])*numpy.nan
        for i in range(y_data.shape[0]):
            y_data[i] = c_k_y[i,0]

        line.set_xdata(x_data)
        line.set_ydata(y_data)

        self.reference_lines.append(line)

        # theta values
        """
        line, = ax.plot([], [],
            lw='1', ls='-', marker='', ms=4, c='r',
            label='c_{k}^{\\theta}'
        )

        x_data = time
        y_data = numpy.ones(time.shape[0])*numpy.nan
        for i in range(y_data.shape[0]):
            y_data[i] = c_k_q[i,0]

        line.set_xdata(x_data)
        line.set_ydata(y_data)
        """

        self.reference_lines.append(line)

        # recalculate x and y limits
        ax.relim()
        ax.autoscale_view()

        # show canvas
        if self.show_canvas:
            self.reference_fig.show()
            plt.pause(1e-8)

    def create_waterfall_plot(self):
        """ create a waterfall like plot of each pattern generator result """
        # assemble plot
        ax = self.waterfall_fig.add_subplot(1,1,1)
        ax.set_ylabel("stuff")
        ax.set_xlabel("time [s]")

        # assemble different trajectories
        self.waterfall_lines = []


        # retrieve data from data structure
        time    = self.data['time']
        C_kp1_x = self.data['C_kp1_x']

        # extend given time by one horizon
        deltaT = time[-1] - time[-2]
        self.waterfall_time = numpy.zeros((time.shape[0] + C_kp1_x.shape[1],))
        self.waterfall_time[:time.shape[0]] = time
        last_horizon = numpy.arange(0, deltaT*C_kp1_x.shape[1],0.1) + deltaT
        self.waterfall_time[time.shape[0]:] = last_horizon + time[-1]

        # create for each instance a trajectory of the states
        for i in range(C_kp1_x.shape[0]):
            label = ''
            if i == 0:
                label = 'C_{k+1}^{x}'
            line, = ax.plot([], [],
                lw='1', ls='-', marker='', ms=4, c='r', label=label
            )

            dummy = numpy.ones(self.waterfall_time.shape[0])*numpy.nan
            a = i; b = i + C_kp1_x.shape[1]
            dummy[a:b] = C_kp1_x[i,:]

            line.set_xdata(self.waterfall_time)
            line.set_ydata(dummy)

            self.waterfall_lines.append(line)

        # recalculate x and y limits
        ax.relim()
        ax.autoscale_view()

        # show canvas
        if self.show_canvas:
            self.waterfall_fig.show()
            plt.pause(1e-8)


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

