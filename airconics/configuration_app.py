# -*- coding: utf-8 -*-
# MainWindow class and most of this file are edited from OCC.Display.SimpleGui
# @Author: p-chambers
# @Date:   2016-08-23 14:43:28
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-12-08 19:16:46
import logging
import os
import sys

from OCC import VERSION
from OCC.Display.backend import load_backend, get_qt_modules
from airconics.topology import Topology_GPTools
import matplotlib.pyplot as plt
import numpy as np
import itertools
from deap import tools

log = logging.getLogger(__name__)

# Currently only trying qt
used_backend = load_backend()
log.info("GUI backend set to: {0}".format(used_backend))


from OCC.Display.qtDisplay import qtViewer3d
QtCore, QtGui, QtWidgets, QtOpenGL = get_qt_modules()

from matplotlib.backends.backend_qt4agg import (
    FigureCanvasQTAgg as FigureCanvas)

from airconics.matplotlib_radar import radar_factory, example_data


class Airconics_Viewgrid(QtWidgets.QWidget):
    """A simple grid containing both a 3d viewer and a range of performance
    metrics for the geometry contained in the widget

    Inputs
    ------
    Topology - airconics.Toplogy (default None)
        The Topology to display in this widget: see attributes. If no Topology
        is specified. An empty Topology will be created

    Attributes
    ----------
    Topology - airconics.Topology object
        The aircraft topology object which is mapped to this viewer. This is
        intended to not be deleted.

    Notes
    -----
    """

    # Note: Some of these have a min target, some have max... misleading
    data_labels = ['Static Margin', 'Fuel Burn',
                   'Cost', 'Weight', 'Range', 'Payload']

    colors = itertools.cycle(['b', 'r', 'g', 'm', 'y'])

    def __init__(self, topology=None, *args):
        super(Airconics_Viewgrid, self).__init__()

        # Create a blank topology object if one has not been provided
        if topology:
            self._Topology = topology

        # Matplotlib colour character (different for each instance)
        self.color = next(self.colors)

        grid = QtGui.QGridLayout(self)
        self.setLayout(grid)
        viewer = qtViewer3d(*args)

        viewer.setMinimumSize(200, 200)

        self.viewer = viewer

        grid.setSpacing(10)
        grid.setMargin(10)

        # Add the viewer spanning 3/4 of the width of the widget
        grid.addWidget(viewer, 0, 0, 1, 1)

        self.InitDataCanvas()

        # Add the canvas to a new VBox layout with a title
        data_group = QtGui.QGroupBox("Estimated Performance Metrics")
        data_box = QtGui.QVBoxLayout(data_group)

        data_box.addWidget(self._data_canvas)

        data_group.setLayout(data_box)

        grid.addWidget(data_group, 0, 1)

        self.select_button = QtGui.QPushButton('Select', self)

        grid.addWidget(self.select_button, 1, 0, 1, 2)

    @property
    def Topology(self):
        return self._Topology

    @Topology.setter
    def Topology(self, newTopology):
        self.viewer._display.EraseAll()
        self._Topology = newTopology
        self._Topology.Display(self.viewer._display)
        self.viewer._display.FitAll()

        # This initialises some data in the radar plot: remove this later!
        data = np.random.random(len(self.data_labels))

        self._fillpolygon.set_xy(zip(self.radar_factory, data))

        self._fig.canvas.draw()

    # @QtCore.pyqtSlot()
    # def onSelectButtonClick(self):
    #     Airconics_Viewgrid.select_clicked.emit()

    # @QtCore.pyqtSlot()
    # def Evolve(self):
        # May be able to migrate this to Topology.setter

        # self._ax.redraw_in_frame()

    def InitDataCanvas(self):
        """Initialises a radar chart in self._data_canvas to be embedded in
        the parent viewer widget

        The radar chart contains the labels defined at the class level via
        self.data_labels.
        """
        # Labels
        # labels = []
        # outputs = []

        # data_group = QtGui.QGroupBox("Estimated Performance Metrics")
        # data_gridlayout = QtGui.QVBoxLayout(data_group)

        # for i, lbl_string in enumerate(self.data_box_labels):
        #     label = QtGui.QLabel(lbl_string)
        #     labels.append(label)

        #     # output = QtGui.QLineEdit("Nil")
        #     # output.setReadOnly(True)
        #     # outputs.append(output)

        #     data_gridlayout.addWidget(label)
        #     # data_gridlayout.addWidget(output, i, 1)

        # data_group.setLayout(data_gridlayout)
        Nvars = len(self.data_labels)
        # Nvars = len(self.data_labels)
        self.radar_factory = radar_factory(Nvars, frame='polygon')

        # This initialises some data in the radar plot: remove this later!
        data = np.random.random(Nvars)

        self._fig = plt.figure(facecolor="white")
        self._ax = self._fig.add_subplot(111, projection='radar')

        self._ax.set_rgrids([0.2, 0.4, 0.6, 0.8])
        self._ax.set_rmin(0.)
        self._ax.set_rmax(1.)

        self._fillpolygon = self._ax.fill(self.radar_factory, data,
                                          facecolor=self.color,
                                          edgecolor=self.color, linewidth=2,
                                          alpha=0.25)[0]
        self._ax.set_rgrids([0.2, 0.4, 0.6, 0.8])
        self._ax.set_rmin(0.)
        self._ax.set_rmax(1.)
        self._ax.set_varlabels(self.data_labels)

        # plt.tight_layout()

        self._data_canvas = FigureCanvas(self._fig)
        self._data_canvas.setParent(self)
        self._data_canvas.setFocusPolicy(QtCore.Qt.StrongFocus)

        # self._data_canvas.setMinimumSize(200, 200)
        self._data_canvas.setMaximumSize(200, 200)

    def selManual(individuals, k):
        """Randomly select *k* individuals from the input *individuals* using *k*
        tournaments of *tournsize* individuals. The list returned contains
        references to the input *individuals*.

        :param individuals: A list of individuals to select from.
        :param k: The number of individuals to select.
        :param tournsize: The number of individuals participating in each tournament.
        :returns: A list of selected individuals.

        This function uses the :func:`~random.choice` function from the python base
        :mod:`random` module.
        """
        # Will need to wait for input signal (selection click) here
        return None


class MainWindow(QtWidgets.QMainWindow):
    """The main Aircraft Topology (configuration) App.

    A number of 3D AirconicsViewgrids (equal to NX * NY) will be created in
    this mainwindow widget. Select buttons of each widget are connected to
    a single slot, such that all topologies rebuild (i.e., are 'evolved') from
    the selected parent.

    Selection of visibly 'fit' aircraft models should be based on both the
    information displayed in the radar chart in the widget, and the users
    sensible judgement based on manufacturability and feasibility.

    Parameters
    ----------
    size : tuple of scalar, length 2 (default 1024 768)
        The (x, y) size of the main window widget, in pixels

    NX, NY : int
        The number of widgets (geometry viewers) in the horizontal/vertical
        direction

    Topologies : list of airconics.Topology
        The topologies to be used in each of the viewers (default empty list)

    Notes
    -----
    This is currently in development, and currently no topology evolution is
    performed.
    """
    global_select_clicked = QtCore.pyqtSignal()

    def __init__(self, size=(1024, 768),
                 NX=2,
                 NY=2,
                 Topologies=[],
                 cxpb=0.5,
                 mutpb=0.2,
                 *args):
        QtWidgets.QMainWindow.__init__(self, *args)
        self.setWindowTitle(
            "occ-airconics aircraft topology app ('%s' backend)" %
            (used_backend))

        # Set up the main widget (this will have several widgets nested within)
        self.main_widget = QtGui.QWidget(self)
        self.setCentralWidget(self.main_widget)

        grid = QtGui.QGridLayout(self.main_widget)
        self.setLayout(grid)

        self.N = NX * NY    # The total number of widgets (and geometries)
        self.cpxb = cxpb
        self.mutpb = mutpb

        # Set up the grid (i, j) widget layout
        positions = [(i, j) for i in range(NX) for j in range(NY)]

        # Add the sub widgets for evolved topology options (9 for now?)
        self.viewer_grids = []

        # create the global population and some randomly selected individuals
        self.topo_tools = Topology_GPTools(max_levels=4)
        self.Init_Evolution()

        for i, position in enumerate(positions):
            viewer_grid = Airconics_Viewgrid()
            grid.addWidget(viewer_grid, *position)

            # connect the select button from the viewer grid to the signal
            viewer_grid.select_button.clicked.connect(
                self.global_select_clicked)

            self.viewer_grids.append(viewer_grid)

        # Connect the main signal to the rebuild function
        self.global_select_clicked.connect(self.EvolveInteractive)

        if not sys.platform == 'darwin':
            self.menu_bar = self.menuBar()
        else:
            # create a parentless menubar
            # see: http://stackoverflow.com/questions/11375176/qmenubar-and-qmenu-doesnt-show-in-mac-os-x?lq=1
            # noticeable is that the menu ( alas ) is created in the
            # topleft of the screen, just
            # next to the apple icon
            # still does ugly things like showing the "Python" menu in
            # bold
            self.menu_bar = QtWidgets.QMenuBar()
        self._menus = {}
        self._menu_methods = {}

        # place the window in the center of the screen, at half the
        # screen size
        self.centerOnScreen()

        self.resize(size[0], size[1])

    def centerOnScreen(self):
        '''Centers the window on the screen.'''
        resolution = QtWidgets.QDesktopWidget().screenGeometry()
        self.move((resolution.width() / 2) - (self.frameSize().width() / 2),
                  (resolution.height() / 2) - (self.frameSize().height() / 2))

    def add_menu(self, menu_name):
        _menu = self.menu_bar.addMenu("&" + menu_name)
        self._menus[menu_name] = _menu

    def add_function_to_menu(self, menu_name, _callable):
        check_callable(_callable)
        try:
            _action = QtWidgets.QAction(
                _callable.__name__.replace('_', ' ').lower(), self)
            # if not, the "exit" action is now shown...
            _action.setMenuRole(QtWidgets.QAction.NoRole)
            _action.triggered.connect(_callable)

            self._menus[menu_name].addAction(_action)
        except KeyError:
            raise ValueError('the menu item %s does not exist' % menu_name)

    @QtCore.pyqtSlot()
    def EvolveInteractive(self):
        """Based on the DEAP eaSimple, but using a hybrid interaction
        selection method


        """
        # This will produce N individuals from N tournaments, each with a size
        # of 2

        # individuals = selTournament(k=self.N, tournsize=2)
        # for viewer in self.viewer_grids:
            # viewer.Topology = 
        return self.sender()

    def Init_Evolution(self, verbose=__debug__):
        """"""
        self.topo_tools._init_population(n=100)

        self.hof = tools.HallOfFame(1)

        self.stats = tools.Statistics(lambda ind: ind.fitness.values)
        self.stats.register("avg", np.mean)
        self.stats.register("std", np.std)
        self.stats.register("min", np.min)
        self.stats.register("max", np.max)

        self.logbook = tools.Logbook()
        self.logbook.header = ['gen', 'nevals'] + self.stats.fields

        self.hof.update(self.topo_tools.population)

        # record = self.stats.compile(self.topo_tools.population)
        # self.logbook.record(gen=0, **record)

        # if verbose:
            # log.info(self.logbook.stream)


if __name__ == '__main__':
    from pkg_resources import resource_filename
    app = QtWidgets.QApplication.instance()
    # checks if QApplication already exists
    if not app:  # create QApplication if it doesnt exist
        app = QtWidgets.QApplication(sys.argv)

    # Set up the splash loading screen
    res_pkg = 'airconics.resources.images'
    cover_name = 'cover.png'

    splash_png = resource_filename(res_pkg, cover_name)
    splash_pix = QtGui.QPixmap(splash_png)

    splash = QtGui.QSplashScreen(splash_pix, QtCore.Qt.WindowStaysOnTopHint)
    splash.setMask(splash_pix.mask())

    progressBar = QtGui.QProgressBar(splash)

    splash.show()

    app.processEvents()

    win = MainWindow()

    win.raise_()  # make the application float to the top

    win.show()

    app.processEvents()

    initial_topos = win.topo_tools._toolbox.select(
        win.topo_tools.population, win.N)

    for i, viewer_grid in enumerate(win.viewer_grids):
        viewer_grid.viewer.InitDriver()
        viewer_grid.Topology = win.topo_tools.run(initial_topos[i])
        progressBar.setValue((1./len(win.viewer_grids)) * i * 100)
        app.processEvents()



    # add_menu('primitives')
    # add_function_to_menu('primitives', sphere)
    # add_function_to_menu('primitives', cube)
    # add_function_to_menu('primitives', exit)

    splash.finish(win)


    sys.exit(app.exec_())
