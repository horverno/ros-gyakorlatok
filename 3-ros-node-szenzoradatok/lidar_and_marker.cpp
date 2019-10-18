#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

ros::Publisher pub;
ros::Publisher marker_pub;
visualization_msgs::Marker marker;
bool marker_color = true, prev_marker_color;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud)
{
    pcl::PCLPointCloud2 cloud_filtered;

    // Define min and max for X, Y and Z
    float minX = 0.0, minY = -1.0, minZ = -1.384;
    float maxX = 16.0, maxY = +1.0, maxZ = -0.15;

    pcl::CropBox<pcl::PCLPointCloud2> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(cloud_filtered);
    // publish marker
    prev_marker_color = marker_color;

    if(cloud_filtered.data.size() < 10){
        marker_color = true;
    }
    else{
        marker_color = false;
    }
    if(marker_color){
        marker.color.r = 0.2f;
        marker.color.g = 0.8f;
        marker.color.b = 0.2f;
        marker.color.a = 0.4;
    }
    else{
        marker.color.r = 0.8f;
        marker.color.g = 0.2f;
        marker.color.b = 0.2f;
        marker.color.a = 0.4;
    }
    if(marker_color != prev_marker_color){
        marker_pub.publish(marker);
        ROS_INFO_STREAM("pub");
    }
    // Publish the filtered LIDAR data
    pub.publish(cloud_filtered);

    //ROS_INFO_STREAM(marker_color <<  " - " << prev_marker_color);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "lidar_and_marker");
    ros::NodeHandle nh;

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("points_raw", 1, cloud_cb);

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/velodyne";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  eg. CUBE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 16 / 2; // 16m before car
    marker.pose.position.y = 0;
    marker.pose.position.z = -1.584 / 2; // down 1.584m 
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- eg. 1x1x1 means 1m on a side
    marker.scale.x = 16.0;  // 16m before car
    marker.scale.y = 2.0;
    marker.scale.z = -1.584; // down 1.584m


    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PCLPointCloud2>("points_filt", 1);

    // Spin
    ros::spin();
}
