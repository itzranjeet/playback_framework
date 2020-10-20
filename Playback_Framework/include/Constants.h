
#include <stdint.h>
#include <unordered_map>
#include <functional>

#define COMM_CONFIG_UPDATE 'c'
#define COMM_PLAY_UPDATE 'p'
#define COMM_PUBLISH_MSG_UPDATE 'z' 
#define COMM_RESTART_UPDATE 'r'
#define COMM_PAUSE_UPDATE ' '
#define COMM_NEXT_UPDATE 'n'
#define COMM_BACK_UPDATE 'b'
#define COMM_REFRESH_UPDATE 'R'
#define COMM_QUIT_UPDATE 'q'
namespace playback_framework
{

static constexpr  char kEmittedTrigger[]{"Trigger"};
static constexpr  char kEmittedConfigMsg[]{"ConfigMsg"};
static constexpr  char kEmittedResponse[]{"RosbagMessagerResponse"};
static constexpr char kSensorImageMessageTopic[]{"cam"};
static constexpr char kSensorLidarMessageTopic[]{"lidar"};
static constexpr char kSensorFeatureTopic[]{"Feature"};
static constexpr char kEmmitedImageTopic[]{"cam_rviz"};
static constexpr char kEmmitedLidarTopic[]{"lidar_rviz"};
static constexpr char kEmmitedFeatureTopic[]{"Feature_rviz"};
static constexpr int publisher_queue_size{1};
static constexpr int subscriber_queue_size{1};

typedef enum player_status_t_
{
	CONFIG,
	PLAY,
	PUBLISH,
	RESTART,
        PAUSE,
        NEXT,
        BACK,
        REFRESH,
	QUIT,
}player_status_t;

}
