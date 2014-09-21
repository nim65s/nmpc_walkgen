import os
import sys
import numpy
import json

import matplotlib
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec

class PlotData(object):
    """
    Smart data container for saving plotting relevant data.
    """

    def __init__(self, generator):
        """ build data structures """
        self.data = {}
        self.generator = generator

        # get list keys
        self.hull_keys = generator._hull_keys
        self.plot_keys = generator._plot_keys
        self.data_keys = generator._data_keys

        # fill internal data with keys and empty lists
        for key in self.hull_keys:
            self.data[key] = []

        for key in self.data_keys:
            self.data[key] = []

        for key in self.plot_keys:
            self.data[key] = []

    def update(self):
        """ update internal data from generator """
        for key in self.data:
            val = self.generator.__dict__.get(key, [])
            self.data[key].append(val)

    def reset(self):
        """ reset all internal data """
        for key in self.data:
            self.data[key] = []

    def save_to_file(self, filename=''):
        """
        save data in json format to file

        Parameters
        ----------

        filename: str
            path to output file,
        """
        # generate general filename
        if not filename:
            stamp = strftime("%Y-%m-%d-%H-%M-%S")
            name = '{stamp}_generator_data.json'.format(stamp=stamp)
            filename = os.path.join('/tmp', name)

        # convert numpy arrays into lists
        for key in self.data.keys():
            val = numpy.asarray(self.data[key])
            self.data[key] = val.tolist()

        # save data to file in json format
        with open(filename, 'w') as f:
            json.dump(self.data, f, sort_keys=True, indent=2)


class Plotter(object):
    """
    Real time plotter for pattern generator data. Can create plots online or
    from saved data in json format.
    """

    # counter for pictures
    picture_cnt = 0

    # general mapping for the bird's eye plots
    bird_view_mapping = (
        # CoM
        (
            ('c_k_x',   {'lw':'1', 'ls':'-',  'marker':'.', 'ms':4, 'c':'r', 'label':'$c_{k}$'}),
            ('c_k_y',   {}), # not plotted
            # for rotation
            ('c_k_q',   {}), # not plotted, but used for transformation
        ),
        # Feet
        (
            ('f_k_x',   {'lw':'1', 'ls':'-', 'marker':'x', 'ms':4, 'c':'g', 'label':'$f_{k}^{x}$'}),
            ('f_k_y',   {}),
            # for rotation
            ('f_k_q',   {}),
        ),
        # ZMP
        # TODO how to get current ZMP state?
        (
            ('z_k_x',   {'lw':'1', 'ls':'-', 'marker':'.', 'ms':4, 'c':'b', 'label':'$f_{k}^{x}$'}),
            ('z_k_y',   {}),
        ),
    )

    preview_mapping = (
        # Preview
        (
            ('C_kp1_x', {'lw':'1', 'ls':':', 'marker':'.', 'ms':4, 'c':'r', 'label':'$C_{k+1}^{x}$'}),
            ('C_kp1_y', {}),
            # for rotation
            ('C_kp1_q', {}),
        ),
        (
            ('F_k_x', {'lw':'1', 'ls':':', 'marker':'x', 'ms':4, 'c':'k', 'label':'$F_{k}^{x}$'}),
            ('F_k_y', {}),
            # for rotation
            ('F_k_q', {}),
        ),
        (
            ('Z_kp1_x', {'lw':'1', 'ls':':', 'marker':'.', 'ms':4, 'c':'b', 'label':'$Z_{k+1}$'}),
            ('Z_kp1_y', {}),
        ),
    )

    def __init__(self,
        generator=None, show_canvas=True, save_to_file=False, filename='',
        fmt='png'
    ):
        """
        Real time plotter for pattern generator data. Can create plots online or
        from saved data in json format.

        Parameters
        ----------

        generator: BaseGenerator instance
            A generator instance which enables online plotting.

        show_canvas: bool
            Flag enabling visualization using a matplotlib canvas.

        save_to_file: bool
            Flag enabling saving pictures as files.

        filename: str
            Path to output file. Note that a counter will be added to filename!

        fmt: str
            File format accepted from matplotlib. Defaults to 'png'.

        """
        # see_to_filetting matplotlib to pretty printing
        matplotlib.rc('xtick', labelsize=6)
        matplotlib.rc('ytick', labelsize=6)
        matplotlib.rc('font', size=10)
        matplotlib.rc('text', usetex=True)

        # some plotting options
        self.figsize = (8.0, 5.0)

        # save reference to pattern generator
        # NOTE used for online plotting
        self.generator = generator

        # transformation matrix used to rotated feet hull
        self.T = numpy.zeros((2,2))

        # try to get data from generator or empty data else
        if generator:
            try:
                self.data = self.generator.data.data
            except:
                err_str = 'could not access data in generator, initializing empty data set.'
                sys.stderr.write(err_str + '\n')
                self.data = {}
        else:
            self.data = {}

        # decides if plots are shown in a canvas
        self.show_canvas = show_canvas

        # decides if plots are saved as pictures
        self.save_to_file = save_to_file

        if self.save_to_file:
            # generate general filename
            if not filename:
                stamp = strftime("%Y-%m-%d-%H-%M-%S")
                directory = '{stamp}_plots'.format(stamp=stamp)
                path = os.path.join('/tmp', directory)
                name = 'pattern_plot'
            else:
                path = os.path.dirname(filename)
                name = os.path.basename(filename).strip()
                name = name.split('.')[0]

            # extract path and filename
            self.filepath = path
            self._fname = name
            self._ffmt  = fmt

            # define filename as function from name, counter and format
            def filename(self):
                name = self._fname
                cnt  = self.picture_cnt
                fmt  = self._ffmt
                return '{name}{cnt}.{fmt}'.format(name=name, cnt=cnt, fmt=fmt)

        # BIRD'S EYE VIEW
        # initialize figure with proper size
        self.fig = plt.figure(figsize=self.figsize)

        ax = self.fig.add_subplot(1,1,1)
        ax.set_title('Aerial View')
        ax.set_ylabel('y [m]')
        ax.set_xlabel("x [m]")

        # assemble different trajectories
        self.bird_view_axis  = ax
        self.bird_view_lines = {}

        for item in self.bird_view_mapping:
            # get mapping for x values
            name      = item[0][0]
            settings  = item[0][1]

            # layout line with empty data, but right settings
            line, = ax.plot([], [], **settings)

            # store lines for later update
            self.bird_view_lines[name] = line

        # add current preview to plot
        for item in self.preview_mapping:
            # get mapping for x values
            name      = item[0][0]
            settings  = item[0][1]

            # layout line with empty data, but right settings
            line, = ax.plot([], [], **settings)

            # store lines for later update
            self.bird_view_lines[name] = line

        # show plot canvas with tght layout
        self.fig.tight_layout()
        if self.show_canvas:
            self.fig.show()
            plt.pause(1e-8)

    def _save_to_file(self):
        """ save figure as pdf """
        # convert numpy arrays into lists
        f_path = os.path.join(self.path, self.filename())
        self.fig.savefig(f_path, format='pdf', dpi=self.dpi)
        self.picture_cnt += 1

    def load_from_file(self, filename):
        """
        load data from file

        .. NOTE: Expects data to be in json format!

        Parameters
        ----------

        filename: str
            path to file that is opened via open(filename, 'r')

        """
        # check if file exists
        if not os.path.isfile(filename):
            err_str = 'filename is not a proper path to file:\n filename = {}'.format(filename)
            raise IOError(err_str)

        self.input_filename = filename

        # try to read data
        with open(self.input_filename, 'r') as f:
            self.data = json.load(f)

    def update(self):
        """ creates plot of x/y trajectories on the ground """

        print self.data.keys()
        time = numpy.asarray(self.data['time'])

        # BIRD'S EYE VIEW
        for item in self.bird_view_mapping:
            # get names from mapping
            x_name = item[0][0]
            y_name = item[1][0]
            q_name = None
            # get theta name only if defined
            if len(item) > 2:
                q_name = item[2][0]

            # get line
            line = self.bird_view_lines[x_name]

            # define data
            x_data = numpy.ones(time.shape[0])*numpy.nan
            y_data = numpy.ones(time.shape[0])*numpy.nan

            # get hull data
            # TODO add more
            points = self.data.get('rfoot', numpy.zeros(0))

            # assemble data
            for i in range(time.shape[0]):
                # x value
                val = numpy.asarray(self.data[x_name])
                if len(val.shape) > 1:
                    val = val[i,0]
                else:
                    val = val[i]
                x_data[i] = val

                # y value
                val = numpy.asarray(self.data[y_name])
                if len(val.shape) > 1:
                    val = val[i,0]
                else:
                    val = val[i]
                y_data[i] = val

                # if we plot foot positions draw also foot hull
                if 'f_k' in x_name or 'f_k' in y_name:
                    add_poly = False
                    if i == 0:
                        add_poly = True
                    else:
                        if not x_data[i] == x_data[i-1] \
                        or not y_data[i] == y_data[i-1]:
                            add_poly = True

                    if add_poly:
                        if q_name:
                            val = numpy.asarray(self.data[q_name])
                            if len(val.shape) > 1:
                                val = val[i,0]
                            else:
                                val = val[i]
                            q = val

                        # update transformation matrix
                        T = self.T
                        c = numpy.cos(q); s = numpy.sin(q)
                        T[0,0] = c; T[0,1] = -s
                        T[1,0] = s; T[1,1] =  c

                        lfoot = numpy.asarray(self.data['lfoot'][i])
                        points = numpy.asarray((x_data[i], y_data[i]))

                        # first rotate
                        hull = T.dot(lfoot.transpose()).transpose()
                        hull =hull + points

                        settings = {
                            #'closed'    : None,
                            'fill'      : None,
                            'edgecolor' : 'gray',
                            #'hatch'     : '/',
                        }
                        poly = plt.Polygon(hull, **settings)
                        self.bird_view_axis.add_patch(poly)

            line.set_xdata(x_data)
            line.set_ydata(y_data)

        # PREVIEW
        ## calculate last time increment
        #T = self.data['T'][-1]
        #N = self.data['N'][-1]
#
        ## extend time by one horizon for preview
        #preview_time = numpy.zeros((time.shape[0] + N,))
        #preview_time[:time.shape[0]] = time
        #preview_time[time.shape[0]:] = numpy.arange(0, T*N, T) + T + time[-1]

        for item in self.preview_mapping:
            # get names from mapping
            x_name = item[0][0]
            y_name = item[1][0]
            q_name = None
            # get theta name only when defined
            if len(item) > 2:
                q_name = item[2][0]

            # get line
            line = self.bird_view_lines[x_name]

            # define data
            x_data = numpy.asarray(self.data[x_name][-1])
            y_data = numpy.asarray(self.data[x_name][-1])
            if q_name:
                q_data = numpy.asarray(self.data[q_name][-1])
            else:
                q_data = numpy.zeros(x_data.shape[0])

            print 'x_data', x_data
            print 'y_data', y_data
            print 'q_data', q_data

            # assemble and transform polygons
            points = numpy.asarray(zip(x_data, y_data))
            rfoot = numpy.asarray(self.data['rfoot'][-1])

            # if we plot foot positions draw also foot hull
            if 'F_k' in x_name or 'F_k' in y_name:
                for i in range(points.shape[0]):
                    # update transformation matrix
                    q = q_data[i]
                    T = self.T
                    c = numpy.cos(q); s = numpy.sin(q)
                    T[0,0] = c; T[0,1] = -s
                    T[1,0] = s; T[1,1] =  c

                    # first rotate
                    hull = T.dot(rfoot.transpose()).transpose()

                    # perform translation afterwards
                    hull = hull + points[i]

                    settings = {
                        #'closed'    : None,
                        'fill'      : None,
                        'edgecolor' : 'gray',
                        #'hatch'     : '/',
                    }
                    poly = plt.Polygon(hull, **settings)
                    self.bird_view_axis.add_patch(poly)

            line.set_xdata(x_data)
            line.set_ydata(y_data)

        # AFTERMATH
        # recalculate x and y limits
        self.bird_view_axis.relim()
        self.bird_view_axis.autoscale_view()
        self.bird_view_axis.set_aspect('equal')

        # define legend
        self.bird_view_axis.legend(loc='upper left')

        # show canvas
        if self.show_canvas:
            plt.pause(1e-8)

        if self.save_to_file:
            self._save_to_file()

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
        data_mapping =(
           # x
           ('c_k_x',   {'lw':'1', 'ls':'-',  'marker':'', 'ms':4, 'c':'r', 'label':'$c_{k}^{x}$'}),
           ('f_k_x',   {'lw':'1', 'ls':'--', 'marker':'', 'ms':4, 'c':'b', 'label':'$f_{k}^{x}$'}),
           #('C_kp1_x', {'lw':'1', 'ls':'-', 'marker':'', 'ms':4, 'c':'r', 'label':'$C_{k+1}^{x}$'}),
           #('F_kp1_x', {'lw':'1', 'ls':'-', 'marker':'', 'ms':4, 'c':'r', 'label':'$F_{k+1}^{x}$'}),
           ('Z_kp1_x', {'lw':'1', 'ls':'--', 'marker':'', 'ms':4, 'c':'g', 'label':'$Z_{k+1}^{x}$'}),

           # y
           ('c_k_y',   {'lw':'1', 'ls':'-.',  'marker':'', 'ms':4, 'c':'k', 'label':'$c_{k}^{y}$'}),
           ('f_k_y',   {'lw':'1', 'ls':'- ', 'marker':'', 'ms':4, 'c':'k', 'label':'$f_{k}^{y}$'}),
           #('C_kp1_y', {'lw':'1', 'ls':'-',  'marker':'', 'ms':4, 'c':'r', 'label':'$C_{k+1}^{y}$'}),
           #('F_kp1_y', {'lw':'1', 'ls':'-',  'marker':'', 'ms':4, 'c':'r', 'label':'$F_{k+1}^{y}$'}),
           ('Z_kp1_y', {'lw':'1', 'ls':'-.',  'marker':'', 'ms':4, 'c':'orange', 'label':'$Z_{k+1}^{y}$'}),

           # theta
           #('c_k_q',   {'lw':'1', 'ls':'-',  'marker':'', 'ms':4, 'c':'r', 'label':'$c_{k}^{x}$'}),
           #('f_k_q',   {'lw':'1', 'ls':'-',  'marker':'', 'ms':4, 'c':'r', 'label':'$f_{k}^{x}$'}),
           #('C_kp1_q', {'lw':'1', 'ls':'-', 'marker':'', 'ms':4, 'c':'r', 'label':'$C_{k+1}^{x}$'}),
           #('F_kp1_q', {'lw':'1', 'ls':'-', 'marker':'', 'ms':4, 'c':'r', 'label':'$F_{k+1}^{x}$'}),
        )

        for item in data_mapping:
            name = item[0]
            item = item[1]

            line, = ax.plot([], [], **item)

            x_data = time
            y_data = numpy.ones(time.shape[0])*numpy.nan
            for i in range(time.shape[0]):
                val = self.data[name]
                if len(val.shape) == 2:
                    val = val[i,0]
                else:
                    val = val[i]
                y_data[i] = val

            line.set_xdata(x_data)
            line.set_ydata(y_data)

            self.reference_lines.append(line)

        # recalculate x and y limits
        ax.relim()
        ax.autoscale_view()

        # define legend position
        legend = ax.legend(loc='upper left')

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

        def show(self):
            # use interactive mode for "real time" plotting
            if self.show_canvas:
                plt.interactive(True)
