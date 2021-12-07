#include "deliberative_manager.h"
#include "deliberative_solver_listener.h"
#include <thread>
#include <chrono>

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

    std::thread t([&dm]()
                  {
                      std::chrono::steady_clock::time_point tick_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(tick_duration);
                      while (ros::ok())
                      {
                          dm.tick();
                          ros::spinOnce();
                          std::this_thread::sleep_until(tick_time += std::chrono::milliseconds(tick_duration));
                      }
                  });
    t.join();

    return 0;
}