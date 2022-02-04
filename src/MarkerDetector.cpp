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
#include <webcam2ros/corners.h>

class MarkerDetector
{
protected:

    ros::NodeHandle nh_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Ptr<cv::aruco::GridBoard> board_; 
    cv::Size img_size_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::string distortion_model_;
    bool is_fisheye_;
    std::map<std::string, cv::Mat> pose_map_;
    std::string img_topic_;
    std::string caminfo_topic_;
    bool first = true;

private:


public:

    MarkerDetector();
    MarkerDetector(int dictionary_num);
    
    virtual ~MarkerDetector(){};

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
    ros::Publisher corner_pub_;
};


MarkerDetector::MarkerDetector(int dictionary_num)
    : parameters_(cv::aruco::DetectorParameters::create())
{
    std::cout << "constructor for MarkerDetector" << std::endl;
    image_transport::ImageTransport it(nh_);
    // ros::NodeHandle priv_nh("~");
    // nh_.getParam("img_topic", img_topic_);
    // nh_.getParam("caminfo_topic", caminfo_topic_);
    img_topic_ = "/camera/color/image_raw";
    caminfo_topic_ = "/camera/color/camera_info";
    img_sub_     = it.subscribe(this->img_topic_, 1, &MarkerDetector::img_cb, this);
    caminfo_sub_ = nh_.subscribe(this->caminfo_topic_, 1, &MarkerDetector::caminfo_cb, this);
    corner_pub_=nh_.advertise<webcam2ros::corners>("corners", 100);
    setDictionary(dictionary_num);
    board_ = cv::aruco::GridBoard::create(1, 2, 0.055, 0.01, dictionary_, 0);

}

MarkerDetector::MarkerDetector()
    :MarkerDetector(10)
{}


void MarkerDetector::caminfo_cb(const sensor_msgs::CameraInfoConstPtr& caminfo_msg)
{
    // if (this->img_size_.empty())
    if (first)
    {
        first = false;
        this->img_size_ = cv::Size(caminfo_msg->width, caminfo_msg->height);
        this->distortion_model_ = caminfo_msg->distortion_model;
        // this->camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)caminfo_msg->K.data());
        this->dist_coeffs_ = cv::Mat(4, 1, CV_32F, (void*)caminfo_msg->D.data());

        camera_matrix_.create(3,3,CV_64FC1);
        for(int i{0}; i<9; i++)
        {
            int row = i/3, col=i%3;
            // ROS_INFO("[%d, %d] = %f", row, col, caminfo_msg->K[i]);
            camera_matrix_.at<double>(row, col) = caminfo_msg->K[i];
        }

        ROS_INFO("[image size] width: %d, height: %d", img_size_.width, img_size_.height);
        ROS_INFO("[camera matrix] %f %f %f\n %f %f %f\n %f %f %f", 
                    camera_matrix_.at<double>(0,0),camera_matrix_.at<double>(0,1),camera_matrix_.at<double>(0,2),
                    camera_matrix_.at<double>(1,0),camera_matrix_.at<double>(1,1),camera_matrix_.at<double>(1,2),
                    camera_matrix_.at<double>(2,0),camera_matrix_.at<double>(2,1),camera_matrix_.at<double>(2,2));
        ROS_INFO("[dist coefficients]: %f, %f, %f, %f", 
            dist_coeffs_.at<double>(0,0),dist_coeffs_.at<double>(1,0),dist_coeffs_.at<double>(2,0),dist_coeffs_.at<double>(3,0));

        // std::cout << img_size_ << std::endl;
        // std::cout << distortion_model_ << std::endl;
        // std::cout << camera_matrix_ << std::endl;
        // std::cout << distortion_model_ << std::endl;

    }

}



void MarkerDetector::img_cb(const sensor_msgs::ImageConstPtr& image_msg)
{
    // ros msgs to cv mat
    // std::cout << "image callback" << std::endl;
    cv::Mat frame;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;        
    try{
        frame = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        // cv::imshow("view", frame);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
    }

    // cv::Mat new_frame;
    // this->undistortImage(frame, new_frame, true);
    // if(!new_frame.empty())
    //     cv::imshow("view", new_frame);
    // cv::imshow("view", frame);

    webcam2ros::corners marker_corners;
    marker_corners.header = image_msg->header;

    cv::aruco::detectMarkers(frame, this->dictionary_, markerCorners, markerIds, this->parameters_, rejectedCandidates, camera_matrix_, dist_coeffs_);
    cv::Mat copy = frame.clone();
    if (markerIds.size())
    {
        std::cout << markerIds.size() << " markers detected! ids are ...";
        for(auto& id : markerIds)
        {
            std::cout << id << " ";
        }
        for(auto& corners : markerCorners)
        {
            for(auto& point : corners)
            {
                std::cout << "corner: [" << point.x << ", "<<point.y << "], ";
                marker_corners.corners.push_back(point.x);
                marker_corners.corners.push_back(point.y);
            }
        }
        corner_pub_.publish(marker_corners);
        std::cout << std::endl;
        cv::aruco::drawDetectedMarkers(copy, markerCorners, markerIds);

        cv::Vec3d rvec, tvec;        
        std::vector<cv::Vec3d> rvecs, tvecs;        
        int valid {0};
        
        try{
            valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board_, camera_matrix_, dist_coeffs_, rvec, tvec);
            // cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.055, this->camera_matrix_, this->dist_coeffs_, rvecs, tvecs);

        }
        catch(cv::Exception& e)
        {
            std::cout << "copy size: " << copy.size() << ", channel: " << copy.channels();
            std::cout << ", frame size: " << frame.size() << ", channel: " << frame.channels() << std::endl;
            std::cout << e.what() << std::endl;
            return;
        }

        if(valid)
        {
            std::cout << "found pose of board." << std::endl;
            cv::aruco::drawAxis(copy, this->camera_matrix_, this->dist_coeffs_, rvec, tvec, 0.5);
            std::cout << rvec << " " << tvec << std::endl;
        }
        else
        {
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.04, this->camera_matrix_, this->dist_coeffs_, rvecs, tvecs);
            for (int i{0}; i < rvecs.size(); i++) {
                cv::aruco::drawAxis(copy, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.5);
                std::cout << rvecs[i] << " " << tvecs[i] << std::endl;
            }

        }


        cv::imshow("view", copy);

    }    
    else
        cv::imshow("view", copy);



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


void MarkerDetector::undistortImage(cv::Mat& input, cv::Mat& output, bool is_fisheye)
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
    ros::init(argc, argv, "marker_detector");
    cv::namedWindow("view");
    cv::startWindowThread();
    MarkerDetector mh;
    ros::spin();
    cv::destroyWindow("view");
    return 0;
}

