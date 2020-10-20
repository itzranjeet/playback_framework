namespace platform_communicator{

template<typename M>
void ROSCommunicator::RegisterPublisherToTopic(const std::string& topic, uint32_t size)
{
	if(nullptr != node_handle_)
	{
		publishers_[topic]  = node_handle_->advertise<M>(topic, size);
	}
}

template <typename M>
void ROSCommunicator::PublishMessageToTopic(const M& message, const std::string & topic)
{
	auto it = publishers_.find(topic);
	if(it != publishers_.end())
	{
                ros::Rate loop_rate(3);
		it->second.publish(message);
		loop_rate.sleep();
        }
}


template<typename M, typename N >
void ROSCommunicator::RegisterSubscriberToTopic(const std::string& topic, const uint32_t size, void(N::*callback)(M), N *obj)
{
	if(nullptr != node_handle_)
	{
		subscribers_[topic] = node_handle_->subscribe(topic, size, callback, obj);
	}
}

template<typename M>
void ROSCommunicator::RegisterImagePublisherToTopic(const std::string& topic, uint32_t size)
{
	if(nullptr != it_)
	{
		Image_publishers_  = it_->advertise(topic, size);
        }
}

template <typename M>
void ROSCommunicator::PublishImageMessageToTopic(const M& message, const std::string & topic)
{
                //sensor_msgs::Image Image_msg;
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR16);
		ros::Rate loop_rate(3);
		Image_publishers_.publish(message);
		loop_rate.sleep();
		std::cout<<"Image Publish........"<<std::endl;	
}

} //namespace platform_communicator
