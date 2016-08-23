# -*- coding: utf-8 -*-
# MainWindow class and most of this file are edited from OCC.Display.SimpleGui
# @Author: p-chambers
# @Date:   2016-08-23 14:43:28
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-08-23 16:56:48
import logging
import os
import sys

from OCC import VERSION
from OCC.Display.backend import load_backend, get_qt_modules
from airconics import Topology

log = logging.getLogger(__name__)

# Currently only trying qt
used_backend = load_backend()
log.info("GUI backend set to: {0}".format(used_backend))


from OCC.Display.qtDisplay import qtViewer3d
QtCore, QtGui, QtWidgets, QtOpenGL = get_qt_modules()


# Need to make small changes to the pythonocc qtviewer 3d class:
class Airconics_viewer(qtViewer3d):
    """Same functionality as the standard QT viewer from python occ, but
    standard mouse click event has a different functionality

    The control key is now required to rotate, zoom or pan around the geometry.
    Standard mouse click is now reserved for 'selecting' the best topology for
    evolutionary operations/reproduction

    Attributes
    ----------
    topology - airconics.Topology object
        The aircraft topology object which is mapped to this viewer. This is
        intended to not be deleted.
    """
    # use a shared 

    def init(self, *kwargs):
        super(Airconics_viewer, self).__init__(kwargs)

        self.Topology = airconics.Topology()
        # key press event should not raise an error on pushing control key:
        # self._key_map[] = lambda *args: None

    def mousePressEvent(self, event):
        modifiers = QtGui.QApplication.keyboardModifiers()
        if modifiers == QtCore.Qt.ControlModifier:
            super(Airconics_viewer, self).mousePressEvent(event)
        else:
            # Will eventually add the standard mouse click behaviour that the
            # current frame's topology will be selected to reproduction
            self.Topology.Display(self._display)

    def mouseMoveEvent(self, event):
        modifiers = QtGui.QApplication.keyboardModifiers()
        if modifiers == QtCore.Qt.ControlModifier:
            super(Airconics_viewer, self).mouseMoveEvent(event)
        else:
            # Will eventually add the standard mouse click behaviour that the
            # current frame's topology will be selected to reproduction
            pass


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, size=(1024, 768), NX=3, NY=3, *args):
        """
        Parameters
        ----------
        NX - int
            the number of X widgets in the grid
        NY - int
            the number of Y widgets in the grid
        """

        QtWidgets.QMainWindow.__init__(self, *args)
        self.setWindowTitle("occ-airconics aircraft topology app ('%s' backend)" % (used_backend))
        self.resize(size[0], size[1])

        # Set up the main widget (this will have several widgets nested within)
        self.main_widget = QtGui.QWidget(self)
        self.setCentralWidget(self.main_widget)

        grid = QtGui.QGridLayout(self.main_widget)
        self.setLayout(grid)

        # Set up the grid (i, j) widget layout
        positions = [(i,j) for i in range(NX) for j in range(NY)]


        # Add the sub widgets for evolved topology options (9 for now?)
        self.canvas = []
        for position in positions:
            viewer = Airconics_viewer(self)
            grid.addWidget(viewer, *position)
            viewer.InitDriver()
            self.canvas.append(viewer)


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
            _action = QtWidgets.QAction(_callable.__name__.replace('_', ' ').lower(), self)
            # if not, the "exit" action is now shown...
            _action.setMenuRole(QtWidgets.QAction.NoRole)
            _action.triggered.connect(_callable)

            self._menus[menu_name].addAction(_action)
        except KeyError:
            raise ValueError('the menu item %s does not exist' % menu_name)



if __name__ == '__main__':
    app = QtWidgets.QApplication.instance() 
    # checks if QApplication already exists
    if not app:  # create QApplication if it doesnt exist
        app = QtWidgets.QApplication(sys.argv)

    win = MainWindow()
    win.show()

    if sys.platform != "linux2":
        display.EnableAntiAliasing()



    # Add some stuff to test the app
    from OCC.BRepPrimAPI import BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeBox

    def sphere(display, event=None):
        display.DisplayShape(BRepPrimAPI_MakeSphere(100).Shape(), update=True)

    def cube(display, event=None):
        display.DisplayShape(BRepPrimAPI_MakeBox(1, 1, 1).Shape(), update=True)

    for viewer in win.canvas:
        sphere(viewer._display)
        cube(viewer._display)

    # add_menu('primitives')
    # add_function_to_menu('primitives', sphere)
    # add_function_to_menu('primitives', cube)
    # add_function_to_menu('primitives', exit)

    win.show()
    win.raise_()  # make the application float to the top
    app.exec_()