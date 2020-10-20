/// @copyright Copyright (C) 2019, KPIT
/// @brief SHARED CSV NODE FUNCTIONALITY

#ifndef PLAYBACK_FRAMEWORK_NODE_BASE_H
#define PLAYBACK_FRAMEWORK_NODE_BASE_H

#include "Constants.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "platform_interface.h"
#include <string>

namespace playback_framework
{

    class NodeBase
    {
    public:
        NodeBase() = default;
        NodeBase (const std::string& nodename);
        virtual ~NodeBase() = default;
        virtual void Run(int &argc, char **argv) = 0;
        virtual void RegisterPublishers() const = 0;
        virtual void RegisterSubscribers() = 0;

    protected:
        platform_communicator::PlatformInterface platform_interface_;
         std::string node_name_;
    };

}//namespace playback_framework`

#endif //PLAYBACK_FRAMEWORK_NODE_BASE_H
