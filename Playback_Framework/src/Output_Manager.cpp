#include<iostream>
#include <sstream>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "ros/ros.h"
#include "Constants.h"
#include "Playback_Framework/ConfigMsg.h"
#include <map>

struct Sync 
{ 
   sensor_msgs::Image cam_frame;
   sensor_msgs::PointCloud lidar_frame;
   sensor_msgs::Image feature_frame; 
};
namespace playback_framework
{
class Output_Manager
{
public:
   Output_Manager();
   ros::Publisher cam_pub,feature_pub,lid_pub;
   ros::Subscriber status_sub,camera_sub,lidar_sub,feature_sub,play_sub,pause_sub,restart_sub,quit_sub,publish_sub,Config_msg_sub,
                   Next_sub,Back_sub,Refresh_sub;
   void Publish();
   void Subscribe();
   void configCallback(const Playback_Framework::ConfigMsg& conf_msg);
   void Play_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   void Pause_Callback(const std_msgs::Int32::ConstPtr& msg);
   void Restart_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   void Quit_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   void Publish_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   void status_Callback(const std_msgs::Int32::ConstPtr& );
   void camCallback(const sensor_msgs::Image::ConstPtr& );
   void lidarCallback(const sensor_msgs::PointCloud::ConstPtr&);
   void FeatureCallback(const sensor_msgs::Image::ConstPtr& process_msg);
   void Next_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   void Back_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   void Refresh_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   cv::Mat rostomatformat(sensor_msgs::Image&);
   void syncmodule(std::map<double,sensor_msgs::Image>&,std::map<double,sensor_msgs::PointCloud>&,std::map<double,sensor_msgs::Image>&);
   template <typename T1, typename T2>
   T1 findClosestKey(const std::map<T1, T2> & data, T1 key);
   std::string checkSensorType(const char* input);
private:
   ros::NodeHandle n;
   std::string Mode,Path,modeparameter;
   double Windowsize;
   int delay;
   playback_framework::player_status_t TriggerStatus;   
   cv::Mat mat1,mat2,mat3,cv_image;
   cv_bridge::CvImagePtr cv_ptr;
   std::map<double,sensor_msgs::Image> Cam_Buffer,Processdata;
   std::map<double,sensor_msgs::PointCloud> Lidar_Buffer;
   std::map<double,sensor_msgs::PointCloud>::iterator Lidar_iterator;
   std::map<double,sensor_msgs::Image>::iterator Cam_iterator;
   sensor_msgs::PointCloud ps_cloud;
   sensor_msgs::Image ps_image;
   std::string timestamp;
   double lower,upper;
   std::vector<Sync> sync_Container;
   std::vector<Sync>::iterator sync_iterator,next_iterator;
   std::string primary_sensor;
   std::string primarysensor;
   Sync sync; 
};

Output_Manager::Output_Manager()
{
  Subscribe();
  Publish();
}

void Output_Manager::Publish()
{
  cam_pub = n.advertise<sensor_msgs::Image>(playback_framework::kEmmitedImageTopic, publisher_queue_size);  
  lid_pub = n.advertise<sensor_msgs::PointCloud>(playback_framework::kEmmitedLidarTopic, publisher_queue_size);
  feature_pub = n.advertise<sensor_msgs::Image>(playback_framework::kEmmitedFeatureTopic, publisher_queue_size);
}

void Output_Manager::Subscribe()
{
  Config_msg_sub = n.subscribe(playback_framework::kEmittedConfigMsg, subscriber_queue_size, &Output_Manager::configCallback,this);
  play_sub = n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Output_Manager::Play_Callback,this);
  pause_sub = n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Output_Manager::Pause_Callback,this);
  restart_sub = n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Output_Manager::Restart_Callback,this);
  quit_sub = n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Output_Manager::Quit_Callback,this);
  publish_sub = n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Output_Manager::Publish_Callback,this);
  Next_sub=n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Output_Manager::Next_Callback,this);
  Back_sub=n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Output_Manager::Back_Callback,this);
  Refresh_sub=n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Output_Manager::Refresh_Callback,this);
  camera_sub = n.subscribe(playback_framework::kSensorImageMessageTopic, subscriber_queue_size, &Output_Manager::camCallback,this);
  lidar_sub = n.subscribe(playback_framework::kSensorLidarMessageTopic, subscriber_queue_size, &Output_Manager::lidarCallback,this);
  feature_sub=n.subscribe(playback_framework::kSensorFeatureTopic, subscriber_queue_size, &Output_Manager::FeatureCallback,this);
  status_sub = n.subscribe(playback_framework::kEmittedResponse, subscriber_queue_size, &Output_Manager::status_Callback,this); 
}

void Output_Manager::configCallback(const Playback_Framework::ConfigMsg& conf_msg)
{
   std::cout<<"Config message is received"<<std::endl;
   Mode=conf_msg.Mode;
   Path=conf_msg.Path;
   modeparameter=conf_msg.modeparameter;
   Windowsize=conf_msg.Windowsize;
   delay=conf_msg.delay;
   std::cout<<"Mode:"<<Mode<<std::endl;
   std::cout<<"Path:"<<Path<<std::endl;
   std::cout<<"Modeparameter:"<<modeparameter<<std::endl;
   std::cout<<"Windowsize:"<<Windowsize<<std::endl;
   std::cout<<"Delay:"<<delay<<std::endl;
}

void Output_Manager::Play_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{ 
 if(Trigger_status->data==PLAY)
  {
    TriggerStatus=PLAY;
    std::cout<<'\n'<<"Play Trigger pressed"<<std::endl;
  }
}

void Output_Manager::Pause_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{ 
 if(Trigger_status->data==PAUSE)
  {
    TriggerStatus=PAUSE;
    std::cout<<'\n'<<"Pause Trigger pressed"<<std::endl;
  }
}

void Output_Manager::Restart_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{ 
 if(Trigger_status->data==RESTART)
  {
    TriggerStatus=RESTART;
    std::cout<<'\n'<<"Restart Trigger pressed"<<std::endl;
  }
}

void Output_Manager::Quit_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{ 
 if(Trigger_status->data==QUIT)
  {
     TriggerStatus=QUIT;
     std::cout<<'\n'<<"Quit Trigger pressed"<<std::endl;
     exit(0);
  }
}

void Output_Manager::Publish_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{ 
 if(Trigger_status->data==PUBLISH)
  {
    TriggerStatus=PUBLISH;
    std::cout<<'\n'<<"Publish Trigger pressed"<<std::endl;
    syncmodule(Cam_Buffer,Lidar_Buffer,Processdata);
    std::cout<<'\n'<<"Sync container----"<<std::endl;
    sync_iterator=sync_Container.begin();
     for(int i=0;i<sync_Container.size();i++)
     { 
        
       if(primarysensor=="CAM")
       {
	std::cout<<"PS:"<<sync_Container[i].cam_frame.header.stamp << "  "<<"LD:" << sync_Container[i].lidar_frame.header.stamp 
             <<"  "<< "F:" << sync_Container[i].feature_frame.header.stamp << std::endl;
        timestamp=std::to_string(sync_Container[i].cam_frame.header.stamp.toSec());
        mat1=rostomatformat(sync_Container[i].cam_frame);
        cv::putText(mat1,timestamp,cv::Point(0, mat1.rows/6),cv::FONT_HERSHEY_DUPLEX,1.5,CV_RGB(255, 0,0), 2);
        timestamp=std::to_string(sync_Container[i].feature_frame.header.stamp.toSec());
	mat2=rostomatformat(sync_Container[i].feature_frame);
        cv::putText(mat2,timestamp,cv::Point(0, mat2.rows/6),cv::FONT_HERSHEY_DUPLEX,1.5,CV_RGB(255, 0, 0), 2);
        cv::vconcat(mat1,mat2,mat3);
        sensor_msgs::ImageConstPtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",mat3).toImageMsg();
        feature_pub.publish(*msg);
        lid_pub.publish(sync_Container[i].lidar_frame);

       }
       else if(primarysensor == "LID")
       {
	std::cout<<"PS:"<<sync_Container[i].lidar_frame.header.stamp << "  "<<"CD:" << sync_Container[i].cam_frame.header.stamp 
             <<"  "<< "F:" << sync_Container[i].feature_frame.header.stamp << std::endl;
        timestamp=std::to_string(sync_Container[i].cam_frame.header.stamp.toSec());
        mat1=rostomatformat(sync_Container[i].cam_frame);
        cv::putText(mat1,timestamp,cv::Point(0, mat1.rows/6),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(255, 0, 0), 2);
        timestamp=std::to_string(sync_Container[i].feature_frame.header.stamp.toSec());
	mat2=rostomatformat(sync_Container[i].feature_frame);
        cv::putText(mat2,timestamp,cv::Point(0, mat2.rows/6),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(255, 0, 0), 2);
        cv::vconcat(mat1,mat2,mat3);
        sensor_msgs::ImageConstPtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8",mat3).toImageMsg();
        feature_pub.publish(*msg);
        lid_pub.publish(sync_Container[i].lidar_frame);  
       } 
      sync_iterator++;             
     }
  }
}

void Output_Manager::Next_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{
  if(Trigger_status->data==NEXT)
  {   
    if(sync_iterator!=sync_Container.end())
    {
       sync_iterator=std::next(sync_iterator,1);
       std::cout<<"Next data:"<< std::endl; 
       std::cout<<"PS:"<<(*sync_iterator).cam_frame.header.stamp << "  "<<"LD:" << (*sync_iterator).lidar_frame.header.stamp 
             <<"  "<< "F:" << (*sync_iterator).feature_frame.header.stamp << std::endl;
    }else 
    {
	std::cout<<"At start of cointainer"<<std::endl;
        sync_iterator=sync_Container.begin();
        sync_iterator=std::next(sync_iterator,0);
        std::cout<<"Next data:"<< std::endl; 
        std::cout<<"PS:"<<(*sync_iterator).cam_frame.header.stamp << "  "<<"LD:" << (*sync_iterator).lidar_frame.header.stamp 
             <<"  "<< "F:" << (*sync_iterator).feature_frame.header.stamp << std::endl;
    }
  } 
}

void Output_Manager::Back_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{
  if(Trigger_status->data==BACK)
  {
    if(sync_iterator==sync_Container.end())
    {
      sync_iterator=std::prev(sync_iterator,2);
       std::cout<<"Previous data:"<< std::endl; 
       std::cout<<"PS:"<<(*sync_iterator).cam_frame.header.stamp << "  "<<"LD:" << (*sync_iterator).lidar_frame.header.stamp 
             <<"  "<< "F:" << (*sync_iterator).feature_frame.header.stamp << std::endl;
    }
    else if(sync_iterator!=sync_Container.begin())
    {
       sync_iterator=std::prev(sync_iterator,1);
       std::cout<<"Previous data:"<< std::endl; 
       std::cout<<"PS:"<<(*sync_iterator).cam_frame.header.stamp << "  "<<"LD:" << (*sync_iterator).lidar_frame.header.stamp 
             <<"  "<< "F:" << (*sync_iterator).feature_frame.header.stamp << std::endl;
    }
    else 
    {
	std::cout<<"At start of cointainer"<<std::endl;
        sync_iterator=sync_Container.end();
        sync_iterator=std::prev(sync_iterator,1);
        std::cout<<"Previous data:"<< std::endl; 
        std::cout<<"PS:"<<(*sync_iterator).cam_frame.header.stamp << "  "<<"LD:" << (*sync_iterator).lidar_frame.header.stamp 
             <<"  "<< "F:" << (*sync_iterator).feature_frame.header.stamp << std::endl;
    }
  } 
}

void Output_Manager::Refresh_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{
  if(Trigger_status->data==REFRESH)
  { 
   //code in progess
  } 
}

cv::Mat Output_Manager::rostomatformat(sensor_msgs::Image& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_image=cv_ptr->image;
    return cv_image;
}

void Output_Manager::syncmodule(std::map<double,sensor_msgs::Image>& Cam_Buffer,std::map<double,sensor_msgs::PointCloud>& Lidar_Buffer,std::map<double,sensor_msgs::Image>& Processdata)
{
primarysensor=checkSensorType(modeparameter.c_str());
if(primarysensor=="CAM")
{
  for(Cam_iterator = Cam_Buffer.begin();Cam_iterator != Cam_Buffer.end(); ++Cam_iterator)
  {
      ps_image=(Cam_iterator->second);
      sync.cam_frame=ps_image;
      sync.lidar_frame=Lidar_Buffer[findClosestKey(Lidar_Buffer,Cam_iterator->first)];
      sync.feature_frame=Processdata[findClosestKey(Processdata,Cam_iterator->first)];
      sync_Container.push_back(sync);
   }
}
else if(primarysensor == "LID")
{
  for(Lidar_iterator = Lidar_Buffer.begin();Lidar_iterator != Lidar_Buffer.end(); ++Lidar_iterator)
  {
      ps_cloud=(Lidar_iterator->second);
      sync.lidar_frame=ps_cloud;
      sync.cam_frame=Cam_Buffer[findClosestKey(Cam_Buffer,Lidar_iterator->first)];
      sync.feature_frame=Processdata[findClosestKey(Processdata,Lidar_iterator->first)];
      sync_Container.push_back(sync);
   }   
}
Lidar_Buffer.clear(); 
Cam_Buffer.clear();
Processdata.clear();
}

std::string Output_Manager::checkSensorType(const char* input)
{
	 std::string sensor = input;     
         for (int i=0;i<sensor.length(); i++)
    	 {
		if (sensor.substr(i,3) == "CAM")
		{
		std::cout << "Primary Sensor is: " << sensor.substr(i,3) <<std::endl;
                primary_sensor="CAM";
		}
		else if (sensor.substr(i,3) == "LID")
		{
		primary_sensor="LID";
		std::cout << "Primary Sensor is: " << sensor.substr(i,3)<<std::endl;   
		}
          }   
return primary_sensor; 
}

template <typename T1, typename T2>
T1 Output_Manager::findClosestKey(const std::map<T1, T2> & data, T1 key)
{
    if (data.size() == 0) {
        //throw std::out_of_range("Received empty map.");
    }

    auto lower = data.lower_bound(key);

    if (lower == data.end()) 
        return std::prev(lower)->first;

    if (lower == data.begin())
        return lower->first;

    auto previous = std::prev(lower);
    if ((key - previous->first) < (lower->first - key))
        return previous->first;

    return lower->first;
}

void Output_Manager::status_Callback(const std_msgs::Int32::ConstPtr& status)
{
  if(status->data==PLAY )
  {
    std::cout<<'\n'<<"Waiting for play trigger"<<std::endl;    
  }
  if(status->data==PUBLISH)
  {
    std::cout<<'\n'<<"Waiting for publish trigger"<<std::endl;
  }
  if(status->data==RESTART)
  {
    std::cout<<'\n'<<"Waiting for restart trigger"<<std::endl;
  }
}
void Output_Manager::FeatureCallback(const sensor_msgs::Image::ConstPtr& process_msg)
{
  sensor_msgs::Image image;
  image=*process_msg;
  std::cout<<"Output received from Feature node......"<<image.header.stamp<<std::endl;
  Processdata.insert(std::pair<double,sensor_msgs::Image>(image.header.stamp.toSec(),image));
  
}

void Output_Manager::camCallback(const sensor_msgs::Image::ConstPtr& Image_msg)
{ 
  sensor_msgs::Image image;
  image=*Image_msg;
  std::cout<<"Image received from rosbag manager......"<<image.header.stamp<<std::endl;
  Cam_Buffer.insert(std::pair<double,sensor_msgs::Image>(image.header.stamp.toSec(),image));
} 

void Output_Manager::lidarCallback(const sensor_msgs::PointCloud::ConstPtr& Lidar_msg)
{ 
  sensor_msgs::PointCloud cloud;
  cloud=*Lidar_msg;
  std::cout<<"Lidar received from rosbag manager......"<<cloud.header.stamp<<std::endl;
  Lidar_Buffer.insert(std::pair<double,sensor_msgs::PointCloud>(cloud.header.stamp.toSec(),cloud));
}

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Output_Manager");
  playback_framework::Output_Manager obj;
  ros::spin();
  return 0;
}