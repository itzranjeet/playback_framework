#ifndef PLATFORM_COMMUNICATOR_COMMUNICATOR_H
#define PLATFORM_COMMUNICATOR_COMMUNICATOR_H

#include <string>
#include <vector>
#include <cstdint>
#include <iostream>
namespace platform_communicator
{

	template<typename T>
	class Crtp
	{
	public:
		T& Underlying() { return static_cast<T&>(*this); }
		T const& Underlying() const { return static_cast<T const&>(*this); }
	};

	template<typename T>
	class Communicator  : public Crtp<T>
	{
	public:

	    void Init (int& argc, char** argv, const std::string& name, uint32_t option)
	    {
	    	this->Underlying().InitNode(argc, argv, name, option);
	    }

	    void Start()
	    {
	    	this->Underlying().StartNode();
	    }

	    auto Time(double t)
	    {
	    	return this->Underlying().Time(t);
	    }
 	    /*template<typename S>
	    std::vector<S> RosbagRead(const char* bag_file_name,std::vector<S> sensor_vect)
	    {	
		std::cout<<"entered in communicator........."<<bag_file_name<<std::endl;
	    	return this->Underlying().template RosbagRead<S>(bag_file_name,sensor_vect);
	    }*/

	    template<typename M>
	    void RegisterPublisher(const std::string& topic, uint32_t size)
	    {
	    	this->Underlying().template RegisterPublisherToTopic<M>(topic, size);
	    }

	    template<typename M, typename N >
	    void RegisterSubscriber (const std::string& topic, const uint32_t size, void(N::*callback)(M), N *obj)
	    {
	    	this->Underlying().RegisterSubscriberToTopic(topic,size, callback,obj);
	    }

	    template <typename M>
	    void Publish(const M& message, const std::string & topic)
	    {
		this->Underlying().PublishMessageToTopic(message, topic);
	    }

	    template<typename M>
	    void RegisterImagePublisher(const std::string& topic, uint32_t size)
	    {
	    	this->Underlying().template RegisterImagePublisherToTopic<M>(topic, size);
	    }

	    template <typename M>
	    void ImagePublish(const M& message, const std::string & topic)
	    {
		this->Underlying().PublishImageMessageToTopic(message, topic);
	    }


	}; // Communicator

} //namespace platform_communicator

#endif //PLATFORM_COMMUNICATOR_COMMUNICATOR_H
