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

# rostopic type /odom 
# rostopic type /odom | rosmsg show
# rostopic type /imu 
# rostopic type /imu  | rosmsg show

# /odom  ==> nav_msgs/Odometry
# /imu   ==> sensor_msgs/Imu


class PlotHandler(object):
    def __init__(self, turtle):
        super(PlotHandler, self).__init__()
        pg.setConfigOptions(antialias=True)
        self.turtle = turtle
        self.app = qtgqt.QtGui.QApplication([])
        self.area = darea.DockArea()
        self.win = qtgqt.QtGui.QMainWindow()

    def initializePlot(self):
        self.first_run = True
        white = (200, 200, 200)
        lightgray = (171, 178, 191)
        darkgray = (30, 40, 50)
        dark1 = (40, 44, 52)
        dark2 = (44, 48, 56)
        red = (200, 66, 66); red1b = pg.mkBrush(200, 66, 66, 200)
        blue = (6, 106, 166); blue1b = pg.mkBrush(6, 106, 166, 200)
        green = (16, 200, 166); green1b = pg.mkBrush(16, 200, 166, 200)
        yellow = (244, 244, 160); yellow1b = pg.mkBrush(244, 244, 160, 200)
             
        self.win.setWindowTitle("TurtleBot plotter")
        self.win.resize(1200,600)
        self.win.setCentralWidget(self.area)

        self.dleft1 = darea.Dock("left 1", size = (1,1))  # give this dock minimum possible size
        self.dleft2 = darea.Dock("left 2", size = (500,400)) # size is only a suggestion
        self.dright1 = darea.Dock("right 1", size=(500,200))
        self.dright2 = darea.Dock("right 2", size=(500,200))
        self.area.addDock(self.dleft1, "left")
        self.area.addDock(self.dleft2, "bottom", self.dleft1)
        self.area.addDock(self.dright1, "right")
        self.area.addDock(self.dright2, "below", self.dright1)   ## place dright2 at top edge of dright1
        self.area.moveDock(self.dright2, "below", self.dright1)
        self.wleft1 = pg.LayoutWidget()
        self.accLabel = qtgqt.QtGui.QLabel("No data\n\n")
        self.posLabel = qtgqt.QtGui.QLabel("No data\n\n")
        self.saveBtn = qtgqt.QtGui.QPushButton("Save dock state")
        self.restoreBtn = qtgqt.QtGui.QPushButton("Restore dock state")
        self.clrBtn = qtgqt.QtGui.QPushButton("Clear")
        self.restoreBtn.setEnabled(False)
        self.wleft1.addWidget(self.accLabel, row=0, col=0)
        self.wleft1.addWidget(self.posLabel, row=0, col=1)
        self.wleft1.addWidget(self.clrBtn, row=0, col=2)
        self.wleft1.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        self.dleft1.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.accLabel.setStyleSheet("font-family: Monospace; font: 30pt; background-color: rgb(44, 48, 56)")
        self.dleft1.addWidget(self.wleft1)
        self.state = None
        self.wleft2 = pg.PlotWidget(title="Plot left 2 (bottom)")
        self.wleft2.setAspectLocked(True)
        self.plot_left2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(6,106,166,255))
        self.wleft2.showGrid(x=True, y=True)
        self.wleft2.addItem(self.plot_left2)
        self.dleft2.addWidget(self.wleft2)
        self.clrBtn.clicked.connect(self.clear)
        self.tcurr = pg.TextItem(text="TurtleBot", color = white)
        self.tstart = pg.TextItem(text="Start", color = red)

        self.wright1 = pg.PlotWidget(title="Plot right 1, random adat")
        self.wright1.plot(np.random.normal(size=20))
        self.wright1.showGrid(x=True, y=True)
        self.dright1.addWidget(self.wright1)
        self.wright2 = pg.PlotWidget(title="Plot right")
        self.plot_right2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(200,66,66,255))
        self.wright2.showGrid(x=True, y=True)
        self.wright2.addItem(self.plot_right2)
        self.dright2.addWidget(self.wright2)

        self.cbarX = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(200, 66, 66), width=10))
        self.textX1 = pg.TextItem(text="X", color = red); self.textX1.setPos(1, -0.1)
        self.textX2 = pg.TextItem(text="-", color = red)
        self.wright2.addItem(self.cbarX)
        self.wright2.addItem(self.textX1)
        self.wright2.addItem(self.textX2)

        self.cbarY = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(16, 200, 166), width=10))
        self.textY1 = pg.TextItem(text="Y", color = green); self.textY1.setPos(2, -0.1)
        self.textY2 = pg.TextItem(text="-", color = green)
        self.wright2.addItem(self.cbarY)
        self.wright2.addItem(self.textY1)
        self.wright2.addItem(self.textY2)

        self.cbarZ = pg.PlotCurveItem(pen=pg.mkPen(qtgqt.QtGui.QColor(6, 106, 166), width=10))
        self.textZ1 = pg.TextItem(text="Z", color = blue); self.textZ1.setPos(3, -0.1)
        self.textZ2 = pg.TextItem(text="-", color = blue)
        self.wright2.addItem(self.cbarZ)
        self.wright2.addItem(self.textZ1)
        self.wright2.addItem(self.textZ2)

        self.wright2.setAspectLocked(True)
        self.win.show()

    def updateFirstPlot(self):
        try:

            self.plot_left2.addPoints(self.turtle.odom_data_x, self.turtle.odom_data_y)
            if self.first_run == True:
                self.wleft2.addItem(self.tcurr)
                self.wleft2.addItem(self.tstart)
                self.tstart.setPos(self.turtle.odom_data_x, self.turtle.odom_data_y)
                self.first_run = False
            self.tcurr.setPos(self.turtle.odom_data_x, self.turtle.odom_data_y)

            None
        except:
            None # todo

    def updateSecondPlot(self):
        try:
            self.textX2.setText("%.2f" % (self.turtle.imu_acc_x))
            self.textX2.setPos(1.9, self.turtle.imu_acc_x)
            self.cbarX.setData(np.array([1.8, 1.8]), np.array([self.turtle.imu_acc_x, 0], dtype = np.float))

            self.textY2.setText("%.2f" % (self.turtle.imu_acc_y))
            self.textY2.setPos(2.9, self.turtle.imu_acc_y)
            self.cbarY.setData(np.array([2.8, 2.8]), np.array([self.turtle.imu_acc_y, 0], dtype = np.float))

            self.textZ2.setText("%.2f" % (self.turtle.imu_acc_z))
            self.textZ2.setPos(3.9, self.turtle.imu_acc_z)
            self.cbarZ.setData(np.array([3.8, 3.8]), np.array([self.turtle.imu_acc_z, 0], dtype = np.float))

        except:
            None # todo

    def updateLabels(self):
        self.accLabel.setText("x: %9.6f\ny: %9.6f\nz: %9.6f" % (self.turtle.imu_acc_x, self.turtle.imu_acc_y, self.turtle.imu_acc_z))        
        self.posLabel.setText("x: %9.6f\ny: %9.6f" % (self.turtle.odom_data_x, self.turtle.odom_data_y))          

    def clear(self):
        self.plot_left2.data = self.plot_left2.data[-1:-20:-1]
        self.first_run = True

class TurtleSubscriber(object):
    def __init__(self):
        """
        self.odom_data_x = None
        self.odom_data_y = None
        self.imu_acc_x = None
        self.imu_acc_y = None
        self.imu_acc_z = None
        """
        #rospy.init_node("listener", anonymous=True)
        rospy.Subscriber("/odom", navmsg.Odometry, self.odometryCallBack)
        rospy.Subscriber("/imu", senmsg.Imu, self.imuCallBack)


    def imuCallBack(self, msg):
        self.imu_acc_x = np.array([float(msg.linear_acceleration.x)])
        self.imu_acc_y = np.array([msg.linear_acceleration.y])
        self.imu_acc_z = np.array([msg.linear_acceleration.z])
        #print("imu(xyz):  %8.4f %8.4f %8.4f" % (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))

    def odometryCallBack(self, msg):
        self.odom_data_x = np.array([msg.pose.pose.position.x])
        self.odom_data_y = np.array([msg.pose.pose.position.y])
        #print("odom: %.4f %.4f " % (msg.pose.pose.position.x, msg.pose.pose.position.y))




if __name__ == "__main__":
    import sys
    print(__file__, "- message reader started ... ")
    rospy.init_node("plotter_", anonymous=True)
    turtleSub = TurtleSubscriber()
    ph = PlotHandler(turtleSub)
    ph.initializePlot()
    timer1 = qtgqt.QtCore.QTimer()
    timer1.timeout.connect(ph.updateSecondPlot)
    timer1.start(30)
    timer2 = qtgqt.QtCore.QTimer()
    timer2.timeout.connect(ph.updateFirstPlot)
    timer2.start(50)
    timer3 = qtgqt.QtCore.QTimer()
    timer3.timeout.connect(ph.updateLabels)
    timer3.start(30)


    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()