#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 定义两个窗口名称，一个用于显示彩色图像，一个用于显示深度图像
static const std::string OPENCV_WINDOW_COLOR = "Image color window leomon";
static const std::string OPENCV_WINDOW_DEPTH = "Image depth window leomon";

class ImageConverter
{
  ros::NodeHandle nh_;  // ROS 节点句柄
  image_transport::ImageTransport it_;  // 图像传输对象
  image_transport::Subscriber image_sub_color;  // 彩色图像订阅者
  image_transport::Subscriber image_sub_depth;  // 深度图像订阅者
  image_transport::Publisher image_pub_color;  // 彩色图像发布者
  image_transport::Publisher image_pub_depth;  // 深度图像发布者

public:
  ImageConverter()
    : it_(nh_)
  {
    // 订阅输入的彩色图像流并注册回调函数
    image_sub_color = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    // 订阅输入的深度图像流并注册回调函数
    image_sub_depth = it_.subscribe("/camera/depth/image_rect_raw", 1, &ImageConverter::depthCb, this);

    // 创建两个OpenCV窗口，分别用于显示彩色图像和深度图像
    cv::namedWindow(OPENCV_WINDOW_COLOR);
    cv::namedWindow(OPENCV_WINDOW_DEPTH);
  }

  ~ImageConverter()
  {
    // 销毁两个OpenCV窗口
    cv::destroyWindow(OPENCV_WINDOW_COLOR);
    cv::destroyWindow(OPENCV_WINDOW_DEPTH);
  }

  // 彩色图像回调函数
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;  // 用于存储转换后的OpenCV图像指针
    try
    {
      // 将ROS图像消息转换为OpenCV图像格式，指定编码为BGR8
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      // 如果转换失败，打印错误信息并返回
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // 在图像上绘制一个圆作为示例
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // 更新彩色图像窗口
    cv::imshow(OPENCV_WINDOW_COLOR, cv_ptr->image);
    cv::waitKey(3);  // 等待3毫秒以更新窗口

    // 发布处理后的彩色图像
    // image_pub_color.publish(cv_ptr->toImageMsg());
  }

  // 深度图像回调函数
  void depthCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;  // 用于存储转换后的OpenCV图像指针
    try
    {
      // 将ROS图像消息转换为OpenCV图像格式，指定编码为32位浮点数
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      // 如果转换失败，打印错误信息并返回
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // 在图像上绘制一个圆作为示例
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // 更新深度图像窗口
    cv::imshow(OPENCV_WINDOW_DEPTH, cv_ptr->image);
    cv::waitKey(3);  // 等待3毫秒以更新窗口

    // 发布处理后的深度图像
    // image_pub_depth.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "opencv_image_node");

  // 创建ImageConverter对象
  ImageConverter ic;

  // 进入ROS主循环
  ros::spin();

  return 0;
}