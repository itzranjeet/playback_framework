#include "UI.h"
struct termios cooked, raw;

namespace playback_framework
{
UI::UI(const std::string & nodename): NodeBase {nodename} {}
UI::~UI() {}
void UI::Run(int & argc, char ** argv)
{
    platform_interface_.Init(argc, argv, node_name_);
    RegisterPublishers();
    data=readconfigfile(argv[1]);
    keyLoop();
    platform_interface_.Start();
}
void UI::RegisterPublishers() const
{
    platform_interface_.RegisterPublisher<std_msgs::Int32>(playback_framework::kEmittedTrigger, publisher_queue_size);
    platform_interface_.RegisterPublisher<Playback_Framework::ConfigMsg>(playback_framework::kEmittedConfigMsg, publisher_queue_size);
}
void UI::RegisterSubscribers() {}
void UI::keyLoop()
{
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use 'c' to Read Config Message. 'q' to Stop. 'p' to play. 'z' to Publish. 'r' to Restart.");
    std::cout<<"I am here,please press key from c,p,z,r,and q"<<std::endl;
    for(;;)
    {
    	if(read(kfd, &c, 1) < 0)
    	{
    		perror("read():");
    		exit(-1);
    	}
    	TriggerStatus = getTriggerStatus(c);
    	if(QUIT == TriggerStatus)
    	{
    		flag.data=TriggerStatus;
			platform_interface_.publish(flag, kEmittedTrigger);
            quit(0);
    		break;
    	}else if(CONFIG == TriggerStatus)
    	{
            platform_interface_.publish(data, kEmittedConfigMsg);
    	}else 
    	{
    		flag.data=TriggerStatus;
            platform_interface_.publish(flag, kEmittedTrigger);
    	}
     }
    return;
}
player_status_t UI::getTriggerStatus(char c)
{
	switch(c)
	{
	case COMM_CONFIG_UPDATE:
             ROS_INFO("I am PRESSING C");
             TriggerStatus=CONFIG;
             break;
        case COMM_PLAY_UPDATE:
             ROS_INFO("I am PRESSING P");
             TriggerStatus=PLAY;
             break;
        case COMM_PUBLISH_MSG_UPDATE:
             ROS_INFO("I am PRESSING Z");
             TriggerStatus=PUBLISH;
             break;
        case COMM_RESTART_UPDATE:
	     ROS_INFO("I am PRESSING R");
             TriggerStatus=RESTART;
             break;
        case COMM_PAUSE_UPDATE:
             ROS_INFO("I am PRESSING Space");
             TriggerStatus = PAUSE;
             break;
       case COMM_NEXT_UPDATE:
             ROS_INFO("I am PRESSING N");
             TriggerStatus=NEXT;
             break;
       case COMM_BACK_UPDATE:
             ROS_INFO("I am PRESSING B");
             TriggerStatus=BACK;
             break;
       case COMM_REFRESH_UPDATE:
             ROS_INFO("I am PRESSING Refresh");
             TriggerStatus=REFRESH;
             break;
        case COMM_QUIT_UPDATE:
             ROS_INFO("I am PRESSING Q");
             ROS_DEBUG("quit");
             TriggerStatus=QUIT;
             break;
          default:
              ROS_DEBUG("quit");
              TriggerStatus=QUIT;
              break;
    }
  return TriggerStatus;
}
void UI::quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}
}

int main(int argc, char * argv[]) {
  std::shared_ptr < playback_framework::NodeBase > sensor;
  sensor = std::make_shared < playback_framework::UI > ("UserInterface");
  sensor -> Run(argc, argv);
  return 0;
}

