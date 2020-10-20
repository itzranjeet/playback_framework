#ifndef PLATFORM_COMMUNICATOR_COMMUNICATOR_FACTORY_H
#define PLATFORM_COMMUNICATOR_COMMUNICATOR_FACTORY_H
#include "communicator.h"
#include "ros_communicator.h"

namespace platform_communicator
{

	static auto& getPlatformCommunicator()
	{
		static ROSCommunicator communicator;
		static Communicator<ROSCommunicator>& comm_ = communicator;
		return comm_;
	}

} //namespace platform_communicator

#endif //PLATFORM_COMMUNICATOR_COMMUNICATOR_FACTORY_H
