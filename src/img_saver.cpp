
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  static int frame_count = 0;

  try
  {
    auto frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("view", frame);
    cv::imwrite("/home/hj/Data/frame_" + std::to_string(frame_count++) + ".png", frame);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_viewer");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
