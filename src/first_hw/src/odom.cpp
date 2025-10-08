#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
using namespace std;

double t_recent;

double s;

void distanceCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double dt;
    double ds;
    double dv = 0; // 定义一个变量用于存储X轴速度

    // 读取激光雷达相关的话题/odom，可以获取小车速度和采集时间
    dt = msg->header.stamp.toSec() - t_recent;
    t_recent = msg->header.stamp.toSec();

    // 从小车的里程计数据中获取X轴速度
    dv = msg->twist.twist.linear.x;
    // 如果X轴速度的绝对值小于0.05，则将其设置为0，以减少误差
    if (fabs(dv) < 0.05)
    {
        dv = 0;
    }

    if (dt > 1 || dt < 0)
    { // 消息间隔太远则说明该时间差无效，这里直接舍去
        dt = 0;
    }
    ds = dv * dt * 1.05; // 根据建立的线性模型进行校正
    s += ds;

    //  在ROS的日志中打印X轴速度
    ROS_INFO("X_速度：%.3f m/s ", dv);
    //  在ROS的日志中打印累计的移动距离
    ROS_INFO("里程计的距离为：%.6f m", s);

    // ROS_INFO("v = %.3f  m/s  ,   s = %.3f   m", dv, s);
    return;
}

int main(int argc, char **argv)
{

    // 设置本地化环境,可以打印汉字
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "speed_subscriber");
    ros::NodeHandle n;
    /*
    使用
    ```bash
    roslaunch scout_bringup open_rslidar.launch
    ```
    启动激光雷达，就可以获取/odom消息
    */
    s = 0;
    t_recent = ros::Time::now().toSec();
    ROS_INFO("init s=%f,t_recent=%0.6fs", s, t_recent);
    ros::Subscriber speed_sub = n.subscribe("/odom", 10, distanceCallback);
    ros::spin();
    return 0;
}