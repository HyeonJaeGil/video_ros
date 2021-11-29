
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>

namespace
{

  cv::Mat frame;
  int frame_count = 0;
  std::string img_topic, key_topic, save_dir;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("view", frame);
    // std::cout << frame_count << std::endl;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void keyCallback(const std_msgs::StringConstPtr & msg)
{
  std::cout << (msg->data).data() << std::endl;
  if(msg->data == std::string{'s'} && !frame.empty())
  {    
    // cv::imwrite(std::string{std::getenv("HOME")} + std::string{"/Data/frame_"} + std::to_string(frame_count) + ".png", frame);
    std::string filename = "frame_" + std::to_string(frame_count) + ".png";
    std::cout << filename << std::endl;
    cv::imwrite(save_dir + filename, frame);
    frame_count++;
  }
    
}

 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_viewer");
  ros::NodeHandle nh("~");
  cv::namedWindow("view");
  cv::startWindowThread();
  nh.getParam("img_topic", img_topic);
  nh.getParam("key_topic", key_topic);
  nh.getParam("save_dir", save_dir);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe(img_topic, 1, imageCallback);
  ros::Subscriber key_sub = nh.subscribe(key_topic, 1, keyCallback);
  ros::spin();
  cv::destroyWindow("view");
}
