#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
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
    
    void caminfo_cb(const sensor_msgs::CameraInfo& caminfo_msg);

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
    ros::Subscriber img_sub_;
    ros::Publisher img_pub_;
};


MarkerHandler::MarkerHandler()
    : parameters_(cv::aruco::DetectorParameters::create()),
      img_topic_("/camera/fisheye1/image_raw"), caminfo_topic_("/camera/fisheye1/camera_info")
{
    ros::NodeHandle priv_nh("~");
    img_sub_     = nh_.subscribe(this->img_topic_, 1, &MarkerHandler::img_cb, this);
    caminfo_sub_ = nh_.subscribe(this->caminfo_topic_, 1, &MarkerHandler::caminfo_cb, this);
}


MarkerHandler::MarkerHandler(int dictionary_num)
    : parameters_(cv::aruco::DetectorParameters::create()),
      img_topic_("/camera/fisheye1/image_raw"), caminfo_topic_("/camera/fisheye1/camera_info")
{
    ros::NodeHandle priv_nh("~");
    img_sub_     = nh_.subscribe(this->img_topic_, 1, &MarkerHandler::img_cb, this);
    caminfo_sub_ = nh_.subscribe(this->caminfo_topic_, 1, &MarkerHandler::caminfo_cb, this);
    setDictionary(dictionary_num);
}


void MarkerHandler::caminfo_cb(const sensor_msgs::CameraInfo& caminfo_msg)
{
    this->img_size_ = cv::Size(caminfo_msg.width, caminfo_msg.height);
    this->distortion_model_ = caminfo_msg.distortion_model;
    // this->camera_matrix_(3, 3, CV_8UC1, caminfo_msg.K);
    // this->dist_coeffs_(4, 1, CV_8UC1, caminfo_msg.D);
}



void MarkerHandler::img_cb(const sensor_msgs::ImageConstPtr& image_msg)
{
    // ros msgs to cv mat
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;        
    try{
        auto frame = cv_bridge::toCvShare(image_msg, "mono8")->image;
        cv::imshow("grayscale", frame);
        cv::aruco::detectMarkers(frame, this->dictionary_, markerCorners, markerIds, this->parameters_, rejectedCandidates);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", image_msg->encoding.c_str());
    }

    if (!markerIds.size())
    {
        // cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        // std::vector<cv::Vec3d> rvecs, tvecs;
        // cv::aruco::estimatePoseBoard(markerCorners, markerIds, )
    }    


}

void MarkerHandler::undistortImage(cv::Mat& input, cv::Mat& output, bool is_fisheye)
{
    if(is_fisheye)
    {
        cv::Mat newCamMat;
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify
            (this->camera_matrix_, this->dist_coeffs_, input.size(), cv::Matx33d::eye(), newCamMat, 1);
        cv::fisheye::undistortImage(input, output, this->camera_matrix_, this->dist_coeffs_, newCamMat);
    }
    else
        cv::undistort(input, output, this->camera_matrix_, this->dist_coeffs_);

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_handler");
    MarkerHandler mh;
    ros::spin();
    return 0;
}

