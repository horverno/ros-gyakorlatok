/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// based on: http://wiki.ros.org/ROS/Tutorials

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
#include <cmath>

float gX, gY, lX, lY;

void leafCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  lX = msg->pose.pose.position.x;
  lY = msg->pose.pose.position.y;
}

void gpsCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  gX = msg->pose.pose.position.x;
  gY = msg->pose.pose.position.y;
}

int main(int argc, char **argv)
{
  gX = gY = lX = lY = 0;
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("/gps/odom", 1000, gpsCallback);
  ros::Subscriber sub2 = n.subscribe("/leaf/odom", 1000, leafCallback);
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("/distance", 1000);
  ros::Rate loop_rate(10);
  ROS_INFO("tavlosag_node start");
  while (ros::ok())
  {
    std_msgs::Float32 dist;
    dist.data = sqrt(pow((gX - lX), 2) + pow((gX - lX), 2));
    chatter_pub.publish(dist);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
