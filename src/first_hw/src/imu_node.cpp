
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ROS_INFO("======================imu=====================\n line:%f, %f, %f, \n v: %f, %f, %f,  \n[%f, %f, %f, %f]\n", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
             msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle node;
    ros::Subscriber subimu = node.subscribe("/imu/data_raw", 1000, imuCallback);
    ros::spin();
    return 0;
}