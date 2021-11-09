#include "rclcpp/rclcpp.hpp"

#include "joystick.hpp"
#include "rostalker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto rostalker = std::make_shared<RosTalker>();
  JoystickManager joystick_manager(rostalker, rostalker->get_logger());
  joystick_manager.start();
  rclcpp::spin(rostalker);
  joystick_manager.stop();
  rclcpp::shutdown();
  return 0;
}
