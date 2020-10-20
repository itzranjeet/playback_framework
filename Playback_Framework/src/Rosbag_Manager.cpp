#include <iostream>
#include <sstream>
#include<stdlib.h>
#include <string>                                                      
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h> 
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include <boost/foreach.hpp>
#include <image_transport/image_transport.h>
#include "rosbag/player.h"
 #include <rosbag/macros.h> 
#include "nlohmann/json.hpp"
#include <experimental/filesystem>
#include <fstream>
#include <unistd.h>
#include "Playback_Framework/ConfigMsg.h"
#include "Constants.h"
#define foreach BOOST_FOREACH

volatile playback_framework::player_status_t PlayerStatus;
ros::Time start_time,end_time,cam_time;
namespace fs = std::experimental::filesystem;
namespace playback_framework
{

class Rosbag_Manager
{
public:
   Rosbag_Manager();
   ros::Subscriber Config_msg_sub,play_sub,pause_sub,restart_sub,quit_sub;
   ros::Publisher status_pub,lidar_pub;
   image_transport::Publisher camera_pub;
   rosbag::Bag bag;
   std_msgs::Int32 status;
   std::string Mode,Path,modeparameter;
   double Windowsize;
   int delay;
   void Publish();
   void Subscribe();
   void RosbagRead(rosbag::Bag&,std::string,auto);
   ros::Time bagconfig(rosbag::Bag& bag,auto parameter);
   void configCallback(const Playback_Framework::ConfigMsg& conf_msg);
   void Play_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   void Pause_Callback(const std_msgs::Int32::ConstPtr& msg);
   void Restart_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   void Quit_Callback(const std_msgs::Int32::ConstPtr& Trigger_status);
   std::vector<std::string> topics;
   std::vector<sensor_msgs::PointCloud> Lidar_Buffer;
   int quit;
   rosbag::View view_;
   rosbag::View::iterator m_samples_itrator,next_itrator;

private:
    ros::NodeHandle n;
    image_transport::ImageTransport it;   
};

Rosbag_Manager::Rosbag_Manager():it(n)
{ 
  Subscribe();
  Publish();
}

void Rosbag_Manager::Subscribe()
{
    Config_msg_sub = n.subscribe(playback_framework::kEmittedConfigMsg, subscriber_queue_size, &Rosbag_Manager::configCallback,this);
    play_sub = n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Rosbag_Manager::Play_Callback,this);
    pause_sub = n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Rosbag_Manager::Pause_Callback,this);
    restart_sub = n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Rosbag_Manager::Restart_Callback,this);
    quit_sub = n.subscribe(playback_framework::kEmittedTrigger, subscriber_queue_size, &Rosbag_Manager::Quit_Callback,this);
}

void Rosbag_Manager::Publish()
{
    status_pub = n.advertise<std_msgs::Int32>(playback_framework::kEmittedResponse, publisher_queue_size);
    camera_pub = it.advertise(playback_framework::kSensorImageMessageTopic, publisher_queue_size);
    lidar_pub = n.advertise<sensor_msgs::PointCloud>(playback_framework::kSensorLidarMessageTopic, publisher_queue_size);
}

void Rosbag_Manager::configCallback(const Playback_Framework::ConfigMsg& conf_msg)
{
   std::cout<<"Config message is received"<<std::endl;
   Mode=conf_msg.Mode;
   Path=conf_msg.Path;
   modeparameter=conf_msg.modeparameter;
   Windowsize=conf_msg.Windowsize;
   delay=conf_msg.delay;
   for(int i=0;i<conf_msg.RosbagTopics.size();i++)
   {
	topics.push_back(conf_msg.RosbagTopics[i]);
   }
   std::cout<<"Mode:"<<Mode<<std::endl;
   std::cout<<"Path:"<<Path<<std::endl;
   std::cout<<"Modeparameter:"<<modeparameter<<std::endl;
   std::cout<<"Windowsize:"<<Windowsize<<std::endl;
   std::cout<<"Delay:"<<delay<<std::endl;
   PlayerStatus=PLAY;
   status.data=PlayerStatus;
   status_pub.publish(status);  
   std::cout<<'\n'<<"Rosbag is open for reading"<<std::endl;
   bag.open(Path, rosbag::bagmode::Read);
   start_time=bagconfig(bag,modeparameter);
}

void Rosbag_Manager::Play_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{ 
  try
  {
  if(Trigger_status->data==PLAY)
  { 
     PlayerStatus=PLAY;
     std::cout<<'\n'<<"Play Trigger pressed"<<std::endl; 
     if(Mode=="WS" || Mode=="delay" || Mode=="PS" || Mode=="SBS" || Mode=="ContinueT")
     {
       RosbagRead(bag,Mode,modeparameter);
       PlayerStatus=PUBLISH;
       status.data=PlayerStatus ;
       status_pub.publish(status);
     }    
  }
  }
  catch (rosbag::BagException) 
  {
    std::cout<< "Restart the bag, Press R " << std::endl;
    
    PlayerStatus=RESTART;
    status.data=PlayerStatus;
    status_pub.publish(status);
  }
}

void Rosbag_Manager::Pause_Callback(const std_msgs::Int32::ConstPtr& msg)
{
  try
  {
    if(msg->data==PAUSE)
    {
      PlayerStatus=PAUSE;
    }
  }
  catch (rosbag::BagException) 
  {
     std::cout<< "To play again Rosbag, give Config message by R" << std::endl;
     
     PlayerStatus=RESTART;
     status.data=PlayerStatus;
     status_pub.publish(status);
  }
}

void Rosbag_Manager::Restart_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{ 
 if(Trigger_status->data==RESTART)
  {
    PlayerStatus=RESTART;
    std::cout<<'\n'<<"Restart Trigger pressed"<<std::endl;
    start_time=bagconfig(bag,modeparameter);
    std::cout<<"Restart bag....................."<<start_time<<std::endl;
    RosbagRead(bag,Mode,modeparameter);
  }
}

void Rosbag_Manager::Quit_Callback(const std_msgs::Int32::ConstPtr& Trigger_status)
{ 
 if(Trigger_status->data==QUIT)
  {
     PlayerStatus=QUIT;
     std::cout<<'\n'<<"Quit Trigger pressed"<<std::endl;
     exit(0);
  }
}


ros::Time Rosbag_Manager::bagconfig(rosbag::Bag& bag,auto parameter)
{
        view_.addQuery(bag,rosbag::TopicQuery(topics));
        ros::Time time=view_.getBeginTime();
        std::cout<<"Bag start time:"<<time<<std::endl;
	end_time=view_.getEndTime();
	std::cout<<"Bag end_time:"<<end_time<<std::endl;
        next_itrator = view_.begin(); 
        return time;
}

void Rosbag_Manager::RosbagRead(rosbag::Bag& bag,std::string mode,  auto parameter)
{       
       std::cout<<"Reading the rosbag"<<std::endl;

       if(Mode=="PS")
       {
	    m_samples_itrator = next_itrator;
	    while(m_samples_itrator!= view_.end())
	    {  
		rosbag::MessageInstance next_msg = *m_samples_itrator;
		
		if (next_msg.isType<sensor_msgs::Image>() )
		{           
		  sensor_msgs::ImagePtr msg = next_msg.instantiate<sensor_msgs::Image>();
		  sensor_msgs::Image image=*msg;
		  if(next_msg.getTopic()==parameter)
		  {
                    camera_pub.publish(image);
		    std::cout<<"Publishing primary image............................"<<image.header.stamp<<std::endl;
		    start_time=image.header.stamp;
		    ++m_samples_itrator; 
		    next_itrator=m_samples_itrator;
		    break;
		  }
                  camera_pub.publish(image);
		  std::cout<<"Publishing image...................................."<<image.header.stamp<<std::endl;
		  start_time=image.header.stamp;   
		} 

		if (next_msg.isType<sensor_msgs::PointCloud>())
		{ 
		  sensor_msgs::PointCloudPtr msg = next_msg.instantiate<sensor_msgs::PointCloud>();
		  sensor_msgs::PointCloud cloud=*msg;
		  if(next_msg.getTopic()==parameter)
		  {
		    lidar_pub.publish(cloud);
		    std::cout<<"Publishing primary Lidar............................"<<cloud.header.stamp<<std::endl;
		    start_time=cloud.header.stamp;
		    ++m_samples_itrator; 
		    next_itrator=m_samples_itrator;
		    break;
		  }
		  lidar_pub.publish(cloud);
		  std::cout<<"Publishing Lidar...................................."<<cloud.header.stamp<<std::endl;
		  start_time=cloud.header.stamp;
		}
		++m_samples_itrator; 
		next_itrator=m_samples_itrator;
	     }
        }
        else if(Mode=="SBS")
        {
	   rosbag::View view_(bag,rosbag::TopicQuery(topics),start_time,end_time,false);
           foreach(rosbag::MessageInstance const m, view_)
          { 
	      sensor_msgs::PointCloudPtr msg = m.instantiate<sensor_msgs::PointCloud>();
              if (msg != NULL)
	      {
		sensor_msgs::PointCloud message = *msg;
		if(m.getTime()==start_time)
		{                         
		   lidar_pub.publish(message);
		   std::cout<<"Reading the lidar message from Rosbag:"<<message.header.stamp<<std::endl;
		}
		else if(m.getTime()>start_time)
		{
		   start_time = message.header.stamp; 
		   std::cout<<"Reading.........next pointcloud messgae...."<<message.header.stamp<<std::endl;
		   break;
		}           
	      }
	      sensor_msgs::ImagePtr imagemsg = m.instantiate<sensor_msgs::Image>();
	      if (imagemsg != NULL)
	      {
		 sensor_msgs::Image image= *imagemsg;      
		 if(m.getTime()==start_time)
		 {
		    camera_pub.publish(image);
		    std::cout<<"Reading the image message from Rosbag:"<<image.header.stamp<<std::endl;
		 }
		 else if(m.getTime()>start_time)
		 {
		    start_time = image.header.stamp; 
		    std::cout<<"Reading..... next image message...."<<image.header.stamp<<std::endl;
		    break;
		 }         
	      }      
           }
        }
        else if(Mode=="WS")
        {
         ros::Duration Window(Windowsize);
         ros::Time timewindow=(start_time) + (Window);
         std::cout<<"Reading timewindow   "<<start_time<<" "<<timewindow<<std::endl;
         rosbag::View view(bag,rosbag::TopicQuery(topics),start_time,timewindow,false);
         foreach(rosbag::MessageInstance const m, view)
         {	 
          sensor_msgs::ImagePtr imagemsg = m.instantiate<sensor_msgs::Image>(); 
          if (imagemsg != NULL)
          {
	      sensor_msgs::Image image = *imagemsg;
              camera_pub.publish(image); 
              std::cout<<"Reading the image message from Rosbag:"<<image.header.stamp<<std::endl;
              start_time=image.header.stamp;
          }  
          sensor_msgs::PointCloudPtr cloudmsg = m.instantiate<sensor_msgs::PointCloud>();
          if (cloudmsg != NULL)
          {
	     sensor_msgs::PointCloud cloud = *cloudmsg;
             lidar_pub.publish(cloud); 
             std::cout<<"Reading the cloud message from Rosbag:"<<cloud.header.stamp<<std::endl;
             start_time=cloud.header.stamp;
          }          
        }              
       }
       else if(Mode=="ContinueT")
       {
        rosbag::View view(bag,rosbag::TopicQuery(topics),start_time,end_time);
        foreach(rosbag::MessageInstance const m, view)
        {
         sensor_msgs::PointCloudPtr msg = m.instantiate<sensor_msgs::PointCloud>();
         if (msg != NULL)
         {
           sensor_msgs::PointCloud message = *msg;
	   if(PlayerStatus==PAUSE)
           { 
              start_time=message.header.stamp;
              std::cout<<'\n'<<"Pause trigger pressed"<<std::endl;  
              std::cout<<"Rosbag is paused to play, press P....."<<std::endl; 
              break;
            } 
            else if(PlayerStatus==PLAY)
            { 
               lidar_pub.publish(message);
               start_time=message.header.stamp;
	       std::cout<<"Reading the image message from Rosbag:"<<message.header.stamp<<std::endl;
            }
            else if(PlayerStatus==RESTART)
            { 
                lidar_pub.publish(message);
		std::cout<<"Restarting lidar message from the Rosbag:"<<message.header.stamp<<std::endl;
            }                                                             
         }
         sensor_msgs::ImagePtr imagemsg = m.instantiate<sensor_msgs::Image>();
         if (imagemsg != NULL)
         {
	   sensor_msgs::Image image = *imagemsg;
           if(PlayerStatus==PAUSE)
           { 
               start_time=image.header.stamp;
               std::cout<<'\n'<<"Pause trigger pressed"<<std::endl;  
               std::cout<<"Rosbag is paused to play, press P....."<<std::endl; 
               break;
            }
            else if(PlayerStatus==PLAY)
            {
	       camera_pub.publish(image); 
               start_time=image.header.stamp;
	       std::cout<<"Reading the lidar message from Rosbag:"<<image.header.stamp<<std::endl;
            }
            else if(PlayerStatus==RESTART)
            { 
               camera_pub.publish(image);
	       std::cout<<"Restarting image message from the Rosbag:"<<image.header.stamp<<std::endl;
            }
         }    
        }
       }
       else if(Mode=="delay")
       {
          rosbag::View view(bag,rosbag::TopicQuery(topics),start_time,end_time);
          foreach(rosbag::MessageInstance const m, view)
          {                 
		 ros::Rate loop_rate(delay);		 
		 sensor_msgs::PointCloudPtr msg = m.instantiate<sensor_msgs::PointCloud>();
		 if (msg != NULL)
		 { 
		   sensor_msgs::PointCloud message = *msg;  
                   if(PlayerStatus==PAUSE)
                   { 
                     start_time=message.header.stamp;
                     std::cout<<'\n'<<"Pause trigger pressed"<<std::endl;  
                     std::cout<<"Rosbag is paused to play, press P....."<<std::endl; 
                     break;
                   } 
                   else if(PlayerStatus==PLAY)
                   { 
                     lidar_pub.publish(message);
                     start_time=message.header.stamp;
		     std::cout<<"Reading the image message from Rosbag:"<<message.header.stamp<<std::endl;
                   }
                   else if(PlayerStatus==RESTART)
                   { 
                     if (m.getTime()==start_time)
                     {
                     lidar_pub.publish(message);
                     start_time=message.header.stamp;
		     std::cout<<"Restarting lidar message from the Rosbag:"<<message.header.stamp<<std::endl;
                     }
                   }      		  
		   loop_rate.sleep();                                                             
		 }
		 sensor_msgs::ImagePtr imagemsg = m.instantiate<sensor_msgs::Image>();
		 if (imagemsg != NULL)
		 { 
		   sensor_msgs::Image image = *imagemsg;
                   if(PlayerStatus==PAUSE)
                   { 
                     start_time=image.header.stamp;
                     std::cout<<'\n'<<"Pause trigger pressed"<<std::endl;  
                     std::cout<<"Rosbag is paused to play, press P....."<<std::endl; 
                     break;
                   }
                   else if(PlayerStatus==PLAY)
                   {
		     camera_pub.publish(image); 
                     start_time=image.header.stamp;
		     std::cout<<"Reading the lidar message from Rosbag:"<<image.header.stamp<<std::endl;
                   }
                   else if(PlayerStatus==RESTART)
                   { 
		     if (m.getTime()==start_time)
                     {
                     camera_pub.publish(image);
                     start_time=image.header.stamp;
		     std::cout<<"Restarting image message from the Rosbag:"<<image.header.stamp<<std::endl;
                     }
                   }  
		   loop_rate.sleep();
		 }
           }

       }	
       while(start_time==end_time)
       {
              std::cout<<"Bag at end......Restart the Bag by pressing R......."<<std::endl;             
              bag.close();             
              PlayerStatus=RESTART;
              status.data=PlayerStatus;
              status_pub.publish(status);
              break;
       }            
}

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Rosbag_Manager");
  while(ros::ok())
  {
    ros::AsyncSpinner spinner(0);
    spinner.start();
    playback_framework::Rosbag_Manager obj;
    ros::waitForShutdown();
  }
  return 0;
}