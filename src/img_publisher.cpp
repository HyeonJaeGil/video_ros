
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
 

/* Display two pictures per row */
void mergeImage(cv::Mat &dst, std::vector<cv::Mat> &images)
{
	int imgCount = (int)images.size();
 
	if (imgCount <= 0)
	{
		printf("the number of images is too small\n");
		return;
	}
 
	printf("imgCount = %d\n", imgCount);
 
	 /* Each picture is reduced to the specified size */
	int rows = images.front().rows;
	int cols = images.front().cols;
	for (int i = 0; i < imgCount; i++)
	{
		 cv::resize (images [i], images [i], cv::Size (cols, rows)); // Note the difference: two parameters are functions Size: width and height, width corresponding to cols, rows corresponding to high
	}
 
	 /* Create a new image size
		 High: rows * imgCount / 2
		 Width: cols * 2
	*/
	dst.create(rows * imgCount / 2, cols * 2, CV_8UC3);
 
	for (int i = 0; i < imgCount; i++)
	{
		images[i].copyTo(dst(cv::Rect((i % 2) * cols, (i / 2)*rows, images[0].cols, images[0].rows)));
	}
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
 
  cv::VideoCapture cap;
  cv::Mat frame;
  int deviceID=4;
  // if(argc>1)
	// deviceID=argv[1][0]-'0';
  int apiID = cv::CAP_ANY;
  cap.open(deviceID, apiID);

    //check backend api name
  std::cout << cap.getBackendName() << std::endl;

  if(!cap.isOpened()){
	  std::cerr<<"ERROR! Unable to open camera"<<std::endl;
	  return -1;
  }
 
  //set parameters of the camera
  cap.set(cv::CAP_PROP_BUFFERSIZE, 2);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap.set(cv::CAP_PROP_FPS, 30);

  cv::Mat cameraMatrix = ( cv::Mat_<double>(3,3)<< 4.8325697765087983e+02, 0., 3.2249123845863045e+02, 0.,
       4.8318579181728722e+02, 2.3958406604380093e+02, 0., 0., 1. );
  cv::Mat distCoeffs  = (cv::Mat_<double>(5,1) << -4.5624292545288403e-01, 3.2720310258928254e-01,
       7.5246312212940537e-05, 6.5448087382331561e-04,
       -1.8906774851318395e-01 );

  cv::Mat undistorted;

  ros::Rate loop_rate(30);
  while (nh.ok()) {
	  cap.read(frame);
    // cv::resize(frame, resized, cv::Size(640, 480));  

	  if(!frame.empty()){


      cv::undistort(frame, undistorted, cameraMatrix, distCoeffs);
      std::cout << frame.type() << undistorted.type() << std::endl;

      // cv::imshow( "Frame", undistorted );

      cv::Mat merged; 
      std::vector<cv::Mat> inputVector{frame, undistorted};

      mergeImage(merged, inputVector);

      cv::imshow( "Frame", merged );

      // Press  ESC on keyboard to exit
      char c=(char)cv::waitKey(25);
      if(c==27)
        break;

		  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted).toImageMsg();
		  pub.publish(msg);
      // ROS_INFO("Publsihed!");
  	}
    ros::spinOnce();
    loop_rate.sleep();
  }
}
