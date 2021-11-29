#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <map>
#include <sensor_msgs/CameraInfo.h>


class MarkerHandler
{
protected:

    ros::NodeHandle nh_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Size img_size_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::string distortion_model_;
    bool is_fisheye_;
    std::map<std::string, cv::Mat> pose_map_;
    std::string img_topic_;
    std::string caminfo_topic_;

private:


public:

    MarkerHandler();
    MarkerHandler(int dictionary_num);
    
    virtual ~MarkerHandler(){};

    void img_cb(const sensor_msgs::ImageConstPtr& image_msg);
    
    void caminfo_cb(const sensor_msgs::CameraInfoConstPtr& caminfo_msg);

    void setDictionary(int dictionary_num=cv::aruco::DICT_6X6_250)
    {
        // this->dictionary_ = dictionary;
        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_num);

    }

    auto getDictionary(const std::string& dictionary) const
    {
        return this->dictionary_;
    }

    void undistortImage(cv::Mat& input, cv::Mat& output, bool is_fisheye);

    ros::Subscriber caminfo_sub_;
    image_transport::Subscriber img_sub_;
    ros::Publisher img_pub_;
};


MarkerHandler::MarkerHandler(int dictionary_num)
    : parameters_(cv::aruco::DetectorParameters::create())
{
    std::cout << "constructor for MarkerHandler" << std::endl;
    image_transport::ImageTransport it(nh_);
    // ros::NodeHandle priv_nh("~");
    nh_.getParam("img_topic", img_topic_);
    nh_.getParam("caminfo_topic", caminfo_topic_);
    img_sub_     = it.subscribe(this->img_topic_, 1, &MarkerHandler::img_cb, this);
    caminfo_sub_ = nh_.subscribe(this->caminfo_topic_, 1, &MarkerHandler::caminfo_cb, this);
    setDictionary(dictionary_num);

}

MarkerHandler::MarkerHandler()
    :MarkerHandler(10)
{}


// MarkerHandler::MarkerHandler(int dictionary_num)
//     : parameters_(cv::aruco::DetectorParameters::create()),
//       img_topic_("/camera/fisheye1/image_raw"), caminfo_topic_("/camera/fisheye1/camera_info")
// {
//     ros::NodeHandle priv_nh("~");
//     img_sub_     = nh_.subscribe(this->img_topic_, 1, &MarkerHandler::img_cb, this);
//     caminfo_sub_ = nh_.subscribe(this->caminfo_topic_, 1, &MarkerHandler::caminfo_cb, this);
//     setDictionary(dictionary_num);
// }


void MarkerHandler::caminfo_cb(const sensor_msgs::CameraInfoConstPtr& caminfo_msg)
{
    // if (this->img_size_.empty())
    if (true)
    {
        this->img_size_ = cv::Size(caminfo_msg->width, caminfo_msg->height);
        this->distortion_model_ = caminfo_msg->distortion_model;
        this->camera_matrix_ = cv::Mat(3, 3, CV_32F, (void*)caminfo_msg->K.data());
        this->dist_coeffs_ = cv::Mat(4, 1, CV_32F, (void*)caminfo_msg->D.data());

        // ROS_INFO("[image size] width: %d, height: %d", img_size_.width, img_size_.height);
        // ROS_INFO("[camera matrix] %f %f %f\n %f %f %f\n %f %f %f", 
        //             camera_matrix_.at<double>(0,0),camera_matrix_.at<double>(0,1),camera_matrix_.at<double>(0,2),
        //             camera_matrix_.at<double>(1,0),camera_matrix_.at<double>(1,1),camera_matrix_.at<double>(1,2),
        //             camera_matrix_.at<double>(2,0),camera_matrix_.at<double>(2,1),camera_matrix_.at<double>(2,2));
        // ROS_INFO("[dist coefficients]: %f, %f, %f, %f", 
        //     dist_coeffs_.at<double>(0,0),dist_coeffs_.at<double>(1,0),dist_coeffs_.at<double>(2,0),dist_coeffs_.at<double>(3,0));

        std::cout << img_size_ << std::endl;
        std::cout << distortion_model_ << std::endl;
        std::cout << camera_matrix_ << std::endl;
        std::cout << distortion_model_ << std::endl;

    }

}



void MarkerHandler::img_cb(const sensor_msgs::ImageConstPtr& image_msg)
{
    // ros msgs to cv mat
    std::cout << "image callback" << std::endl;
    cv::Mat frame;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;        
    try{
        frame = cv_bridge::toCvShare(image_msg, "mono8")->image;
        // cv::imshow("view", frame);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", image_msg->encoding.c_str());
    }

    cv::Mat new_frame;
    this->undistortImage(frame, new_frame, true);
    if(!new_frame.empty())
        cv::imshow("view", new_frame);

/*
    cv::aruco::detectMarkers(frame, this->dictionary_, markerCorners, markerIds, this->parameters_, rejectedCandidates);
    cv::Mat copy = frame.clone();
    if (markerIds.size())
    {
        std::cout << "detected!" << std::endl;
        cv::aruco::drawDetectedMarkers(copy, markerCorners, markerIds);

        std::vector<cv::Vec3d> rvecs, tvecs;
        // cv::aruco::estimatePoseBoard(markerCorners, markerIds, 
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.1, this->camera_matrix_, this->dist_coeffs_, rvecs, tvecs);
        for (int i=0; i<markerIds.size(); i++)
        {
            cv::aruco::drawAxis(copy, this->camera_matrix_, this->dist_coeffs_, rvecs[i], tvecs[i], 0.1);
        }
        cv::imshow("view", copy);

    }    
*/

    // ROS_INFO("[image size] width: %d, height: %d", img_size_.width, img_size_.height);
    // ROS_INFO("[camera matrix] %f %f %f", *camera_matrix_.ptr<double>(0), *camera_matrix_.ptr<double>(1), *camera_matrix_.ptr<double>(2));
    // ROS_INFO("[camera matrix] %f %f %f", *camera_matrix_.ptr<double>(3), *camera_matrix_.ptr<double>(4), *camera_matrix_.ptr<double>(5));
    // ROS_INFO("[camera matrix] %f %f %f", *camera_matrix_.ptr<double>(6), *camera_matrix_.ptr<double>(7), *camera_matrix_.ptr<double>(8));

    //             // camera_matrix_.at<double>(0,0),camera_matrix_.at<double>(0,1),camera_matrix_.at<double>(0,2),
    //             // camera_matrix_.at<double>(1,0),camera_matrix_.at<double>(1,1),camera_matrix_.at<double>(1,2),
    //             // camera_matrix_.at<double>(2,0),camera_matrix_.at<double>(2,1),camera_matrix_.at<double>(2,2));
    //             // *camera_matrix_.ptr<double>(3), *camera_matrix_.ptr<double>(4), *camera_matrix_.ptr<double>(5),
    //             // *camera_matrix_.ptr<double>(6), *camera_matrix_.ptr<double>(7), *camera_matrix_.ptr<double>(8));

    // ROS_INFO("[dist coefficients]: %f, %f, %f, %f", 
    //     dist_coeffs_.at<double>(0,0), dist_coeffs_.at<double>(1,0), dist_coeffs_.at<double>(2,0), dist_coeffs_.at<double>(3,0));

}


void MarkerHandler::undistortImage(cv::Mat& input, cv::Mat& output, bool is_fisheye)
{
    if(is_fisheye)
    {
        cv::Mat newCamMat;
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify
            (this->camera_matrix_, this->dist_coeffs_, input.size(), cv::Matx33d::eye(), newCamMat, 1);
        cv::fisheye::undistortImage(input, output, this->camera_matrix_, this->dist_coeffs_, this->camera_matrix_, cv::Size(1200, 1200));
    }
    else
        cv::undistort(input, output, this->camera_matrix_, this->dist_coeffs_);

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_handler");
    cv::namedWindow("view");
    cv::startWindowThread();
    MarkerHandler mh;
    ros::spin();
    cv::destroyWindow("view");
    return 0;
}

