#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Create iterators to read the point cloud data
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    unsigned int num_nonzero_points = 0;

    // Iterate through all points
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // Check if all coordinates are non-zero
        if (*iter_x != 0.0f && *iter_y != 0.0f && *iter_z != 0.0f) {
            ++num_nonzero_points;
        }
    }

    ROS_INFO("Number of non-zero points in the PointCloud2 message: %u", num_nonzero_points);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud2_nonzero_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/unitreel1/cloud", 1, pointCloudCallback);

    ros::spin();

    return 0;
}
