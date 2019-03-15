#!/usr/bin/env python
from __future__ import print_function
import rospy
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import sensor_msgs.msg as senmsg

# rostopic type /odom 
# rostopic type /odom | rosmsg show
# rostopic type /imu 
# rostopic type /imu  | rosmsg show

# /odom  ==> nav_msgs/Odometry
# /imu   ==> sensor_msgs/Imu


def odometryCallBack(msg):
    print("odom(x,y): %8.4f %8.4f " % (msg.pose.pose.position.x, msg.pose.pose.position.y))

def imuCallBack(msg):
    print("imu(xyz):  %8.4f %8.4f %8.4f" % (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))

def listener():

    # In ROS, nodes are uniquely named. The anonymous=True flag means that rospy will choose a unique
    # name for our "listener" node so that multiple listeners can run simultaneously.
    rospy.init_node("listener", anonymous=True)
    #rospy.Subscriber("/odom", navmsg.Odometry, odometryCallBack)
    rospy.Subscriber("/imu", senmsg.Imu, imuCallBack)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    print(__file__, "- message reader started ... ")
listener()