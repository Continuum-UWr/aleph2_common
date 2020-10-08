#include "rostalker.h"

#include <input_msgs/InputMessage.h>
#include <sstream>
#include <list>

struct RosTalkerInternals{

};

void RosTalker::Init(int argc, char ** argv){
    ros::init(argc, argv, "input_manager", ros::init_options::AnonymousName);

    n = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
}

void RosTalker::RegisterDevice(std::weak_ptr<DeviceState> dev) {
    auto device_locked = dev.lock();

    printf("Registered device: %s\n", device_locked->name.c_str());

    topics.push_back(std::make_tuple(
            n->advertise<input_msgs::InputMessage>(std::string("input/") + device_locked->name, 1000),
            dev
            ));
}

void RosTalker::Update(){
    ros::Rate loop_rate(30);

    topics.erase( std::remove_if(topics.begin(), topics.end(),
        [](std::tuple<ros::Publisher, std::weak_ptr<DeviceState>> entry) -> bool
        {
            if(std::get<1>(entry).expired()) {
                printf("Device lost!\n");
                return true;
            }

            auto device = std::get<1>(entry).lock();
            input_msgs::InputMessage msg;

            msg.axes = device->axes;
            msg.buttons = device->buttons;
            msg.buttons_pressed = std::move(device->presses);
            msg.buttons_released = std::move(device->releases);
            std::get<0>(entry).publish(msg);

            return false;
        }
    ), topics.end());

    ros::spinOnce();

    loop_rate.sleep();
}
