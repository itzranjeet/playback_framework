namespace platform_communicator
{
	template<typename M>
	void PlatformInterface::RegisterPublisher(const std::string& topic, uint32_t size) const
	{
		auto &platformcommunicator = getPlatformCommunicator();
		platformcommunicator.template RegisterPublisher<M>(topic, size);
	}

	template<typename M, typename N >
	void PlatformInterface::RegisterSubscriber(const std::string& topic, const uint32_t size, void(N::*callback)(M), N *obj)
	{
		auto &platformcommunicator = getPlatformCommunicator();
		platformcommunicator.RegisterSubscriber(topic, size, callback, obj);
	}

	template <typename M>
	void PlatformInterface::publish(const M& message, const std::string & topic)
	{
		auto &platformcommunicator = getPlatformCommunicator();
		platformcommunicator.Publish(message, topic);
	}

	template<typename M>
	void PlatformInterface::RegisterImagePublisher(const std::string& topic, uint32_t size) const
	{ 	
		auto &platformcommunicator = getPlatformCommunicator();
		platformcommunicator.template RegisterImagePublisher<M>(topic, size);
	}

	template <typename M>
	void PlatformInterface::Imagepublish(const M& message, const std::string & topic)
	{
		auto &platformcommunicator = getPlatformCommunicator();
		platformcommunicator.ImagePublish(message, topic);		
	}


} //namespace platform_communicator



