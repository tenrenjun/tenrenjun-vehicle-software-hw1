
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
static const std::string OPENCV_WINDOW_COLOR = "Image color window";
static const std::string OPENCV_WINDOW_DEPTH = "Image depth window";

void callbackForShowColor(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
 
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_COLOR, cv_ptr->image);
    cv::waitKey(3);
}

// show depth image
void callbackForShowDepth(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_DEPTH, cv_ptr->image);
    cv::waitKey(3);
}

int main(int argc, char** argv) {
    cv::namedWindow(OPENCV_WINDOW_COLOR);
    cv::namedWindow(OPENCV_WINDOW_DEPTH);

    ros::init(argc, argv, "show_color_and_depth_node");
    ros::NodeHandle nh;
    
    auto image_sub_color = nh.subscribe("/camera/color/image_raw", 1, callbackForShowColor);
    auto image_sub_depth = nh.subscribe("/camera/depth/image_rect_raw",1, callbackForShowDepth);
    ros::spin();
    cv::destroyWindow(OPENCV_WINDOW_COLOR);
    cv::destroyWindow(OPENCV_WINDOW_DEPTH);
}