#!/usr/bin/env python
from __future__ import print_function
import rospy
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import sensor_msgs.msg as senmsg
import numpy as np
import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import pyqtgraph.dockarea as darea

# rostopic type /gps/odom
# rostopic type /gps/odom  | rosmsg show
# rostopic type /leaf/odom
# rostopic type /leaf/odom | rosmsg show
# /leaf/odom  ==> nav_msgs/Odometry
# /gps/odom   ==> nav_msgs/Odometry

class PlotHandler(object):
    def __init__(self, leaf):
        super(PlotHandler, self).__init__()
        pg.setConfigOptions(antialias=True)
        self.leaf = leaf
        self.app = qtgqt.QtGui.QApplication([])
        self.area = darea.DockArea()
        self.win = qtgqt.QtGui.QMainWindow()

    def initializePlot(self):
        self.first_run = True
        white = (200, 200, 200)
        red = (200, 66, 66); redB = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blueB = pg.mkBrush(6, 106, 166, 200)
        green = (16, 200, 166); greenB = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellowB = pg.mkBrush(244, 244, 160, 200)
             
        self.win.setWindowTitle("Leaf plotter")
        self.win.resize(1000,800)
        self.win.setCentralWidget(self.area)

        self.dock1 = darea.Dock("dock 1", size = (1,1))  # give this dock minimum possible size
        self.dock2 = darea.Dock("dock 2", size = (500,400)) # size is only a suggestion
        self.area.addDock(self.dock1, "left")
        self.area.addDock(self.dock2, "bottom", self.dock1)
        self.widg1 = pg.LayoutWidget()
        self.lxLabel = qtgqt.QtGui.QLabel("No data")
        self.gxLabel = qtgqt.QtGui.QLabel("No data")
        self.saveBtn = qtgqt.QtGui.QPushButton("Save dock state")
        self.restoreBtn = qtgqt.QtGui.QPushButton("Restore dock state")
        self.clrBtn = qtgqt.QtGui.QPushButton("Clear")
        self.restoreBtn.setEnabled(False)
        self.widg1.addWidget(self.lxLabel, row=0, col=0)
        self.widg1.addWidget(self.gxLabel, row=0, col=1)
        self.widg1.addWidget(self.clrBtn, row=0, col=2)
        self.widg1.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        self.dock1.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.lxLabel.setStyleSheet("font-family: Monospace; font: 30pt; background-color: rgb(44, 48, 56)")
        self.dock1.addWidget(self.widg1)
        self.state = None
        self.widg2 = pg.PlotWidget(title="Plot left 2 (bottom)")
        self.widg2.setAspectLocked(True)
        self.pltGpsOdom = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = blueB)
        self.pltLeafOdom = pg.ScatterPlotItem(size = 10, pen = pg.mkPen(None), brush = redB)
        self.widg2.showGrid(x=True, y=True)
        self.widg2.addItem(self.pltGpsOdom)
        self.widg2.addItem(self.pltLeafOdom)
        self.dock2.addWidget(self.widg2)
        self.clrBtn.clicked.connect(self.clear)
        self.tGps = pg.TextItem(text = "Gps", color = blue)
        self.tLeaf = pg.TextItem(text = "Leaf odom", color = red)
        self.tstart = pg.TextItem(text = "Start", color = white)
        self.win.show()

    def updateFirstPlot(self):
        self.pltGpsOdom.addPoints(self.leaf.gps_x, self.leaf.gps_y)
        self.pltLeafOdom.addPoints(self.leaf.leaf_x, self.leaf.leaf_y)
        if self.first_run == True:
            self.widg2.addItem(self.tGps)
            self.widg2.addItem(self.tLeaf)
            self.widg2.addItem(self.tstart)
            self.tstart.setPos(self.leaf.gps_x, self.leaf.gps_y)
            self.first_run = False
        self.tGps.setPos(self.leaf.gps_x, self.leaf.gps_y)
        self.tLeaf.setPos(self.leaf.leaf_x, self.leaf.leaf_y)
        self.gxLabel.setText("x: %9.6f\ny: %9.6f" % (self.leaf.gps_x, self.leaf.gps_y))          

    def clear(self):
        self.pltGpsOdom.data = self.pltGpsOdom.data[-1:-20:-1]
        self.pltLeafOdom.data = self.pltLeafOdom.data[-1:-20:-1]
        self.first_run = True

class LeafSubscriber(object):
    def __init__(self):
        rospy.Subscriber("/gps/odom", navmsg.Odometry, self.gpsOdomCallBack)
        rospy.Subscriber("/leaf/odom", navmsg.Odometry, self.leafOdomCallBack)

    def leafOdomCallBack(self, msg):
        self.leaf_x = np.array([msg.pose.pose.position.x])
        self.leaf_y = np.array([msg.pose.pose.position.y])

    def gpsOdomCallBack(self, msg):
        self.gps_x = np.array([msg.pose.pose.position.x])
        self.gps_y = np.array([msg.pose.pose.position.y])

if __name__ == "__main__":
    import sys
    print(__file__, "- message reader started ... ")
    rospy.init_node("leafplotter", anonymous=True)
    leafSub = LeafSubscriber()
    ph = PlotHandler(leafSub)
    ph.initializePlot()
    timer1 = qtgqt.QtCore.QTimer()
    timer1.timeout.connect(ph.updateFirstPlot)
    timer1.start(30)
    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()
