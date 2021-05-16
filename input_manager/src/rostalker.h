#ifndef ROSTALKER_H
#define ROSTALKER_H

#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

struct DeviceState {
  std::string name;
  std::vector<float> axes;
  std::vector<uint8_t> buttons;
  std::vector<int> presses;
  std::vector<int> releases;
  bool active;
  int buttons_c;
};

class RosTalker {
 public:
  static RosTalker& Instance() {
    static RosTalker inst;
    return inst;
  }

  void Init(int argc, char** argv);
  void RegisterDevice(std::weak_ptr<DeviceState> dev);
  void Update();
  void PublishDevices(bool shutdown = false);
  RosTalker(RosTalker const&) = delete;
  void operator=(RosTalker const&) = delete;

 private:
  std::unique_ptr<ros::NodeHandle> n;
  std::unique_ptr<ros::NodeHandle> pn;
  std::list<std::tuple<ros::Publisher, std::weak_ptr<DeviceState>>> topics;
  ros::Publisher pub;
  std::string n_name;
  RosTalker(){};
};

#endif /* ROSTALKER_H */
