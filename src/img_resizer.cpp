#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

image_transport::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  cv::Mat new_frame;
  try
  {
    auto frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::resize(frame, new_frame, cv::Size2i(640, 480));
    cv::imshow("view", new_frame);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", new_frame).toImageMsg();
  pub.publish(out_msg);

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_resizer");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  std::string input_topic, output_topic;
  nh.getParam("input_topic", input_topic);
  nh.getParam("output_topic", output_topic);
  image_transport::Subscriber sub = it.subscribe(input_topic, 1, imageCallback);
  static image_transport::Publisher pub = it.advertise(output_topic, 1);
  ros::spin();
  cv::destroyWindow("view");
}