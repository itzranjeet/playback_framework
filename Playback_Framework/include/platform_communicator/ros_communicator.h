#ifndef PLATFORM_COMMUNICATOR_ROS_COMMUNICATOR_H
#define PLATFORM_COMMUNICATOR_ROS_COMMUNICATOR_H

#include "communicator.h"
#include <unordered_map>
#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <rosbag/view.h> 
#include <boost/foreach.hpp> 
#define foreach BOOST_FOREACH
namespace platform_communicator
{
	class ROSCommunicator : public Communicator<ROSCommunicator> 
	{
	public:
		void InitNode(int &argc, char **argv, const std::string &name, uint32_t options)
		{
		   ros::init(argc,argv,name,options);
		   node_handle_ = std::make_unique<ros::NodeHandle>();
                   it_=std::make_unique<image_transport::ImageTransport>(*node_handle_);
		}

		void StartNode()
		{
		   ros::spin();
		}

		ros::Time Time(double t)
                {
		   ros::Time Time(t);
		return Time;
		}
         
		template<typename M>
		void RegisterPublisherToTopic(const std::string& topic, uint32_t size);

		template <typename M>
		void PublishMessageToTopic(const M& message, const std::string & topic);

		template<typename M, typename N >
		void RegisterSubscriberToTopic(const std::string& topic, const uint32_t size, void(N::*callback)(M), N *obj);

		template<typename M>
		void RegisterImagePublisherToTopic(const std::string& topic, uint32_t size);

		template <typename M>
		void PublishImageMessageToTopic(const M& message, const std::string & topic);

		template<typename M>
		void RegisterPointPublisherToTopic(const std::string& topic, uint32_t size);


	private:
		std::unique_ptr<ros::NodeHandle>  node_handle_; 
		std::unique_ptr<image_transport::ImageTransport>  it_;   		
		std::unordered_map<std::string, ros::Publisher> publishers_;
		std::unordered_map<std::string, ros::Subscriber> subscribers_;
		image_transport::Publisher Image_publishers_;
		image_transport::Publisher Image_publishers1_;
	};

}//namespace platform_communicator

#include "ros_communicator_impl.h"

#endif //PLATFORM_COMMUNICATOR_ROS_COMMUNICATOR_H
