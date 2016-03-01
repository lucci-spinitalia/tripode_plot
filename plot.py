__author__ = 'luca'
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import sys
import threading


class ALMA_Plot (threading.Thread):
    motor1_position_step = np.zeros(1000, dtype=float)
    motor2_position_step = np.zeros(1000, dtype=float)
    motor3_position_step = np.zeros(1000, dtype=float)
    motor4_position_step = np.zeros(1000, dtype=float)
    time = np.arange(-10,0,0.01,dtype=float)

    curve1 = None
    curve2 = None
    curve3 = None
    curve4 = None

    p1 = None
    vb = None
    vLine = None
    #hLine = None
    label = None

    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)

        self.threadID = threadID
        self.name = name
        self.counter = counter

    def run(self):
        win = pg.GraphicsWindow()
        win.setWindowTitle('Motor Position')

        self.p1 = win.addPlot()
        self.curve1 = self.p1.plot(self.motor1_position_step)

        win.nextRow()
        self.curve2 = win.addPlot().plot(self.motor2_position_step)

        win.nextRow()
        self.curve3 = win.addPlot().plot(self.motor3_position_step)

        win.nextRow()
        self.curve4 = win.addPlot().plot(self.motor4_position_step)

        #cross hair
        self.label = pg.LabelItem(justify='right')
        win.addItem(self.label)

        self.vLine = pg.InfiniteLine(angle=90, movable=False)
        #self.hLine = pg.InfiniteLine(angle=0, movable=False)
        self.p1.addItem(self.vLine, ignoreBounds=True)
        #self.p1.addItem(self.hLine, ignoreBounds=True)

        self.vb = self.p1.vb
        proxy = pg.SignalProxy(self.p1.scene().sigMouseMoved, rateLimit=60, slot=self.mouseMoved)


        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.plot_view_update)
        timer.start(100)

        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

    def mouseMoved(self, evt):
        pos = evt[0]  ## using signal proxy turns original arguments into a tuple

        if self.p1.sceneBoundingRect().contains(pos):
            mousePoint = self.vb.mapSceneToView(pos)
            index = int(mousePoint.x())
            #if index > 0 and index < len(data1):
            self.label.setText("<span style='font-size: 12pt'>x=%0.1f,   <span style='color: green'>y2=%0.1f</span>" % (mousePoint.x(),self.motor1_position_step[index]))
            self.vLine.setPos(mousePoint.x())
            #self.hLine.setPos(mousePoint.y())

    def plot_view_update(self):
        if self.curve1 is not None and self.curve2 is not None and self.curve3 is not None and self.curve4 is not None:

            time = self.time
            self.curve1.setData(time, self.motor1_position_step)
            #self.curve1.setPos(self.time[-1], 0)

            self.curve2.setData(time, self.motor2_position_step)
            #self.curve2.setPos(time, 0)

            self.curve3.setData(time, self.motor3_position_step)
            #self.curve3.setPos(time, 0)

            self.curve4.setData(time, self.motor4_position_step)
            #self.curve4.setPos(time, 0)

    def plot_update(self, x, mot1, mot2, mot3, mot4):

        self.time[:-1] = self.time[1:]
        self.time[-1] = self.time[-2] + x

        self.motor1_position_step[:-1] = self.motor1_position_step[1:]  # shift data in the array one sample left
                                    # (see also: np.roll)
        self.motor1_position_step[-1] = mot1

        self.motor2_position_step[:-1] = self.motor2_position_step[1:]  # shift data in the array one sample left
                                    # (see also: np.roll)
        self.motor2_position_step[-1] = mot2

        self.motor3_position_step[:-1] = self.motor3_position_step[1:]  # shift data in the array one sample left
                                    # (see also: np.roll)
        self.motor3_position_step[-1] = mot3

        self.motor4_position_step[:-1] = self.motor4_position_step[1:]  # shift data in the array one sample left
                                     # (see also: np.roll)
        self.motor4_position_step[-1] = mot4

    def test_plot(self, x):
        self.time[:-1] = self.time[1:]
        self.time[-1] = self.time[-2] + x

        self.motor1_position_step[:-1] = self.motor1_position_step[1:]  # shift data in the array one sample left
                                    # (see also: np.roll)

    def cleanup(self):
        QtGui.QApplication.instance().exit()