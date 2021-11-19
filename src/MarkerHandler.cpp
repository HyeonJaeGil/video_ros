#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <map>



class MarkerHandler
{
protected:
    ros::NodeHandle nh_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    std::map<std::string, cv::Mat> pose_map_;
    std::string topic_name_;

private:


public:
    MarkerHandler();
    MarkerHandler(int dictionary_num);
    
    virtual ~MarkerHandler(){};

    void img_cb(const sensor_msgs::ImageConstPtr& image_msg);

    void setDictionary(int dictionary_num=cv::aruco::DICT_6X6_250)
    {
        // this->dictionary_ = dictionary;
        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_num);

    }

    auto getDictionary(const std::string& dictionary) const
    {
        return this->dictionary_;
    }


    ros::Subscriber sub_;
    ros::Publisher pub_;
};


MarkerHandler::MarkerHandler()
    : parameters_(cv::aruco::DetectorParameters::create()),
      topic_name_("/camera/fisheye1/image_raw")
{
    ros::NodeHandle priv_nh("~");
    sub_ = nh_.subscribe(this->topic_name_, 1, &MarkerHandler::img_cb, this);
}


MarkerHandler::MarkerHandler(int dictionary_num)
    : parameters_(cv::aruco::DetectorParameters::create()),
      topic_name_("/camera/fisheye1/image_raw")
{
    ros::NodeHandle priv_nh("~");
    sub_ = nh_.subscribe(this->topic_name_, 1, &MarkerHandler::img_cb, this);
    setDictionary(dictionary_num);
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
    


}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_handler");
    MarkerHandler mh;
    ros::spin();
    return 0;
}

