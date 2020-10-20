#include<iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "Constants.h"

class Image_processing 
{
public:
    ros::Subscriber sub;
    image_transport::Subscriber sub_image;
    image_transport::Publisher image_pub_;
    Image_processing();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg1);
    template <typename dataType>
    dataType feature_extraction(const dataType& msg1) ;
private:
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cv_image;
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    sensor_msgs::Image msg;
};

Image_processing::Image_processing():it(n)
{
    sub_image = it.subscribe(playback_framework::kSensorImageMessageTopic, 1, &Image_processing::imageCallback,this);
    image_pub_ = it.advertise(playback_framework::kSensorFeatureTopic, 1);
}

void Image_processing::imageCallback(const sensor_msgs::ImageConstPtr& msg1)
{    
    try
    { 
        sensor_msgs::Image image;
        image=feature_extraction(*msg1);
        std::cout<<'\n'<<"Image received.............."<<image.header.stamp<<std::endl;
        image_pub_.publish(image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg1->encoding.c_str());
    }
}
template <typename dataType>
dataType Image_processing::feature_extraction(const dataType& msg) 
{   
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_image=cv_ptr->image;
    cv::Point2f pc(cv_image.cols/2., cv_image.rows/2.);
    cv::Mat r  = cv::getRotationMatrix2D( pc, -180, 1.0);
    cv::warpAffine(cv_image, cv_image, r, cv_image.size());
    sensor_msgs::ImageConstPtr msg_=cv_bridge::CvImage(std_msgs::Header(), "bgr8",cv_image).toImageMsg();
    sensor_msgs::Image outmsg=*msg_;
    outmsg.header.stamp=msg.header.stamp;
    return outmsg;   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Feature_Node");
  Image_processing obj;
  ros::spin();
  return 0;
}
