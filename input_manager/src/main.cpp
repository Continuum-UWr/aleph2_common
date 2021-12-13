#include <climits>
#include <unistd.h>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"

#include "joystick.hpp"
#include "rostalker.hpp"

char hostname[HOST_NAME_MAX];

static std::shared_ptr<RosTalker> rostalker;

void MyShutDown(int)
{
  rostalker->shutdown();
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  gethostname(hostname, HOST_NAME_MAX);

  std::string node_name("input_manager");
  node_name += std::string("_") + std::string(hostname);
  std::replace_if(
    node_name.begin(), node_name.end(),
    [](char x) -> bool {
      return !(std::isalnum(x) || x == '_');
    }, '_');

  rostalker = std::make_shared<RosTalker>(node_name);
  JoystickManager joystick_manager(hostname, rostalker,
    rostalker->get_logger().get_child("JoystickManager"));

  signal(SIGINT, MyShutDown);

  joystick_manager.start();
  rclcpp::spin(rostalker);
  joystick_manager.stop();

  rclcpp::shutdown();
  return 0;
}
