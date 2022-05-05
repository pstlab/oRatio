#include "deliberative_manager.h"

using namespace ratio;

// the duration of each tick in milliseconds..
constexpr size_t tick_duration = 1000;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "deliberative_tier");
    ros::NodeHandle nh;
    ROS_INFO("Starting the Deliberative Tier..");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    deliberative_manager dm(nh);

    ros::Timer tick_timer = nh.createTimer(ros::Duration(1.0), std::bind(&ratio::deliberative_manager::tick, &dm));
    ros::spin();

    return 0;
}