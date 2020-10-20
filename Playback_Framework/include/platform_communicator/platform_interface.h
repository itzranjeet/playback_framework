#ifndef PLATFORM_COMMUNICATOR_PLATFORM_INTERFACE_H
#define PLATFORM_COMMUNICATOR_PLATFORM_INTERFACE_H
#include <string>
#include <vector>
#include <cstdint>
#include "communicator_factory.h"

namespace platform_communicator
{
	class PlatformInterface {
	public:

		PlatformInterface(){}

		~PlatformInterface(){}

		void Init (int & argc, char** argv, const std::string& name)
		{
			uint32_t option = 0;
			auto &platformcommunicator = getPlatformCommunicator();
			platformcommunicator.Init(argc, argv, name, option);
		}

		void Init (int& argc, char** argv, const std::string& name, uint32_t option)
		{
			auto &platformcommunicator = getPlatformCommunicator();
			platformcommunicator.Init(argc, argv, name, option);
		}

		void Start()
		{
			auto &platformcommunicator = getPlatformCommunicator();
			platformcommunicator.Start();
		}

	        auto Time(double t)
		{
			auto &platformcommunicator = getPlatformCommunicator();
			 return platformcommunicator.Time(t);
		}
               
		template<typename M>
		void RegisterPublisher(const std::string& topic, uint32_t size) const;

		template<typename M, typename N>
		void RegisterSubscriber (const std::string& topic, const uint32_t size, void(N::*callback)(M), N *obj);

		template <typename M>
		void publish(const M& message, const std::string & topic);

		template <typename M>
		void Imagepublish(const M& message, const std::string & topic);

		template<typename M>
		void RegisterImagePublisher(const std::string& topic, uint32_t size) const;
	};

}//namespace platform_communicator

#include "platform_interface_impl.h"

#endif// PLATFORM_COMMUNICATOR_PLATFORM_INTERFACE_H
