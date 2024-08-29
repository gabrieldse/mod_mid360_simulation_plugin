#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

// Callback function for PointCloud2 messages
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("Received a PointCloud2 message");
    ROS_INFO("Data type: %s", typeid(msg).name());
}

// Callback function for Imu messages
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_INFO("Received an Imu message");
    ROS_INFO("Data type: %s", typeid(msg).name());
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "sensor_subscriber_node");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Subscribe to the /unilidar/cloud topic
    ros::Subscriber cloud_sub = nh.subscribe("/unilidar/cloud", 10, pointCloudCallback);

    // Subscribe to the /unilidar/imu topic
    ros::Subscriber imu_sub = nh.subscribe("/unilidar/imu", 10, imuCallback);

    // Spin to process incoming messages
    ros::spin();

    return 0;
}
