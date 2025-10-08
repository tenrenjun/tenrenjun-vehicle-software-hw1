
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
static const std::string OPENCV_WINDOW_COLOR = "Image color window";
static const std::string OPENCV_WINDOW_DEPTH = "Image depth window";
 
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_color;
  image_transport::Subscriber image_sub_depth;
  image_transport::Publisher image_pub_color;
  image_transport::Publisher image_pub_depth;
 
public:
  ImageConverter()
    : it_(nh_)
  {
    
 
    
    // cv::namedWindow(OPENCV_WINDOW_DEPTH);
  }
 
  ~ImageConverter()
  {
    
    
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
 
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
 
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_COLOR, cv_ptr->image);
    cv::waitKey(3);
 
    // Output modified video stream
    image_pub_color.publish(cv_ptr->toImageMsg());
  }

  void show_depth() {
    image_sub_depth = it_.subscribe("/camera/depth/image_rect_raw",1,&ImageConverter::depthCb, this);
    cv::namedWindow(OPENCV_WINDOW_DEPTH);
  }

  void show_color() {
    image_sub_color = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    cv::namedWindow(OPENCV_WINDOW_COLOR);
  }
  void depthCb(const sensor_msgs::ImageConstPtr &msg)
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

     // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_DEPTH, cv_ptr->image);
    cv::waitKey(3);
    //
    image_pub_depth.publish(cv_ptr->toImageMsg());
  }

  void destoryWindow(bool t) {
      if(t) {
        cv::destroyWindow(OPENCV_WINDOW_DEPTH);
      }else {
        cv::destroyWindow(OPENCV_WINDOW_COLOR);
      }
  }
};
