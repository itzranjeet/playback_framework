#ifndef PLAYBACK_FRAMEWORK_UI_H
#define PLAYBACK_FRAMEWORK_UI_H

#include <std_msgs/Int32.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "node_base.h"
#include "InputReader.h"

namespace playback_framework
{
   class UI : public NodeBase
   {
	   public:
	     UI(const std::string& nodename);
	     virtual ~UI();
	     virtual void RegisterPublishers() const override;
             virtual void RegisterSubscribers()  override;
	     virtual void Run(int &argc, char **argv) override;
	     Playback_Framework::ConfigMsg data;
	   private:
	     void keyLoop();
	     void quit(int sig);
	     player_status_t getTriggerStatus(char);
	     int kfd = 0;
             
	   protected:
             const std::string topic_name_;
	     std_msgs::Int32 flag;
	     player_status_t TriggerStatus;
	     char c;
    };
}
#endif // PLAYBACK_FRAMEWORK_UI_H
