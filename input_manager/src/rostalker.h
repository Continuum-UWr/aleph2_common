

#ifndef ROSTALKER_H
#define ROSTALKER_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <vector>
#include <memory>
#include <tuple>
#include <list>

struct DeviceState{
    std::string name;
    std::vector<float> axes;
    std::vector<uint8_t> buttons;
    std::vector<int> presses;
    std::vector<int> releases;

    int buttons_c;
};

class RosTalker
{
    public:
        static RosTalker& Instance()
        {
            static RosTalker inst;
            return inst;
        }

        void Init(int argc, char ** argv);
        void RegisterDevice(std::weak_ptr<DeviceState> dev);
        void Update();

        RosTalker(RosTalker const&)      = delete;
        void operator=(RosTalker const&) = delete;

    private:
        std::unique_ptr<ros::NodeHandle> n;

        std::list<std::tuple<ros::Publisher, std::weak_ptr<DeviceState>>> topics;
        RosTalker() {};
};

#endif /* ROSTALKER_H */

