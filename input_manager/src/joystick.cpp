#include <cmath>
#include <functional>
#include <list>

#include <SDL2/SDL.h>
#include <SDL2/SDL_events.h>
#include <SDL2/SDL_joystick.h>

#include <ros/ros.h>
#include <cctype>

#include <libudev.h>
#include <XmlRpcValue.h>

#include "joystick.h"
#include "rostalker.h"

#define DEADZONE 0.10f

char hostname[HOST_NAME_MAX];
static XmlRpc::XmlRpcValue yaml_joysticks;
static std::map<std::string, std::string> joysticks_mapping;

void JoystickManager::Init() {
  SDL_Init(SDL_INIT_JOYSTICK);
  gethostname(hostname, HOST_NAME_MAX);
  ros::param::get("~joysticks", yaml_joysticks);
  
  ROS_ASSERT(yaml_joysticks.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int i = 0; i < yaml_joysticks.size(); i++) {
    XmlRpc::XmlRpcValue &p = yaml_joysticks[i];
    joysticks_mapping[static_cast<std::string>(p["serial"])] = static_cast<std::string>(p["name"]);
  }
}

float AxisValue(float value) {
  float axis_value = value / (value < 0 ? 32768.0f : 32767.0f);

  if (std::abs(axis_value) > DEADZONE) {
    axis_value -= (std::signbit(axis_value) ? -1.0f : 1.0f) * DEADZONE;
    axis_value /= (1.0f - DEADZONE);
  } else
    axis_value = 0.0f;
  return axis_value;
}

void JoystickManager::Update() {
  SDL_Event event;

  while (SDL_PollEvent(&event)) {
    int dev_id = event.jdevice.which;
    if (devices.find(dev_id) == devices.end() &&
        event.type != SDL_JOYDEVICEADDED) {
      ROS_WARN_THROTTLE(1.0, "Received an event for lost device");
      continue;
    }

    switch (event.type) {
      case SDL_JOYAXISMOTION: {
        devices[dev_id]->axes[event.jaxis.axis] = AxisValue(event.jaxis.value);
        break;
      }

      case SDL_JOYBUTTONDOWN:
        devices[dev_id]->buttons[event.jbutton.button] = true;
        devices[dev_id]->presses.push_back(event.jbutton.button);
        break;
      case SDL_JOYBUTTONUP:
        devices[dev_id]->buttons[event.jbutton.button] = false;
        devices[dev_id]->releases.push_back(event.jbutton.button);
        break;

      case SDL_JOYHATMOTION: {
        int hat_position = devices[dev_id]->buttons_c + event.jhat.which * 4;

        for (int i = hat_position; i < hat_position + 4; i++)
          devices[dev_id]->buttons[i] = false;

        if (event.jhat.value & SDL_HAT_UP) {
          devices[dev_id]->buttons[hat_position + 0] = true;
          devices[dev_id]->presses.push_back(hat_position + 0);
        }

        if (event.jhat.value & SDL_HAT_DOWN) {
          devices[dev_id]->buttons[hat_position + 1] = true;
          devices[dev_id]->presses.push_back(hat_position + 1);
        }

        if (event.jhat.value & SDL_HAT_LEFT) {
          devices[dev_id]->buttons[hat_position + 2] = true;
          devices[dev_id]->presses.push_back(hat_position + 2);
        }

        if (event.jhat.value & SDL_HAT_RIGHT) {
          devices[dev_id]->buttons[hat_position + 3] = true;
          devices[dev_id]->presses.push_back(hat_position + 3);
        }

        break;
      }

      case SDL_JOYDEVICEADDED:
        NewDevice(event.jdevice.which);
        break;
      case SDL_JOYDEVICEREMOVED:
        DeviceLost(event.jdevice.which);
        break;
    }
  }
}

void JoystickManager::NewDevice(int joy_id) {
  auto dev = SDL_JoystickOpen(joy_id);
  auto state = std::make_shared<DeviceState>();
  auto inst_id = SDL_JoystickInstanceID(dev);
  auto guid = SDL_JoystickGetDeviceGUID(joy_id);

  uint16_t guid_short = 0;
  for (int i = 0; i < 16; i++) guid_short += guid.data[i] * i;

  state->axes.resize(SDL_JoystickNumAxes(dev));
  state->buttons_c = SDL_JoystickNumButtons(dev);
  state->buttons.resize(SDL_JoystickNumButtons(dev) +
                        SDL_JoystickNumHats(dev) * 4);
  
  /*
  Exorcizamus te, omnis immundus spiritus, omnis satanica potestas, 
  omnis incursio infernalis adversarii, omnis legio, omnis congregatio et secta diabolica. 
  Ergo, omnis legio diabolica, adiuramus te…cessa decipere humanas creaturas, 
  eisque æternæ perditionìs venenum propinare… Vade, satana, inventor et magister omnis fallaciæ, 
  hostis humanæ salutis…Humiliare sub potenti manu Dei; contremisce et effuge, 
  invocato a nobis sancto et terribili nomine…quem inferi tremunt…Ab insidiis diaboli, libera nos, Domine. 
  Ut Ecclesiam tuam secura tibi facias libertate servire, te rogamus, audi nos.
  */

  //kurwa do zmiany jak wejdzie SDL2 i funkcja SDL_GetSerial
  int *hw_data = *(int **) ((void *)dev + 136);
  int *item = *(int **) ((void *)hw_data + 8);
  char *path = *(char **)((void*)item+8);

  ROS_DEBUG("path from struct: %s", path);

  state->name = JoystickManager::GetSerial(path);

  std::replace_if(
      state->name.begin(), state->name.end(),
      [](char x) -> bool { return !(isalnum(x) || x == '_'); }, '_');

  for (int i = 0; i < state->buttons_c; i++) {
    state->buttons[i] = SDL_JoystickGetButton(dev, i);
  }
  for (unsigned int i = 0; i < state->axes.size(); i++) {
    state->axes[i] = AxisValue(SDL_JoystickGetAxis(dev, i));
  }

  devices.insert({inst_id, state});
  RosTalker::Instance().RegisterDevice(state);
}

std::string JoystickManager::GetSerial(char *path) {
  /* udev */
  struct udev *udev;
  struct udev_enumerate *enumerate;
  struct udev_list_entry *u_devices, *dev_list_entry;
  struct udev_device *u_device;
  const char *serial_id = "";
  std::string name = "";

  /* Create the udev object */
	udev = udev_new();
	if (!udev) {
		ROS_ERROR("Couldn't create new udev context");
		exit(1);
	}
	
	/* Create a list of the devices in the 'input' subsystem. */
	enumerate = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(enumerate, "input");
	udev_enumerate_scan_devices(enumerate);
	u_devices = udev_enumerate_get_list_entry(enumerate);
	udev_list_entry_foreach(dev_list_entry, u_devices) {	
		/* Get first device path in the sys system */
    const char *sys_path = udev_list_entry_get_name(dev_list_entry);
    /* Create new udev device */
		u_device = udev_device_new_from_syspath(udev, sys_path);

    ROS_DEBUG("sys path: %s", sys_path);

    if(!u_device) {
      ROS_DEBUG("Couldn't create new udev device: %s", sys_path);
      udev_device_unref(u_device);
      continue;
    }
      
    /* Get '/dev' device path */
    const char *dev_path = udev_device_get_devnode(u_device);
    if(!dev_path) {
      ROS_DEBUG("Couldn't get the device '/dev' path");
      udev_device_unref(u_device);
      continue;
    }
    
    if(std::strcmp(dev_path, path)) {
      udev_device_unref(u_device);
      continue;
    }
    
    ROS_DEBUG("dev path: %s", dev_path);
    
    /* Get deivce's usb parent */
		u_device = udev_device_get_parent_with_subsystem_devtype(u_device, "usb","usb_device");
    
    if(!u_device) {
      ROS_DEBUG("Couldn't get device usb parent");
      udev_device_unref(u_device);
      continue;
    }

    serial_id = udev_device_get_sysattr_value(u_device, "serial");
		
    udev_device_unref(u_device);
	}
	/* Free the enumerator object */
	udev_enumerate_unref(enumerate);
  /* Free the udev context */
	udev_unref(udev);

  // for(auto elem: joysticks_mapping) {
  //   ROS_DEBUG("map test: %s, %s", elem.first.c_str(), elem.second.c_str());
  // }

  //get the name by serial
  ROS_DEBUG("serial id: %s", serial_id);
  //ROS_ASSERT(serial_id);
  ROS_DEBUG("count: %d", joysticks_mapping.count(serial_id));
  
  if(joysticks_mapping.find(serial_id) != joysticks_mapping.end())
    name = joysticks_mapping.at(serial_id);

  return name;
}

void JoystickManager::DeviceLost(int id) {
  ROS_INFO("Removed device: %s", devices[id]->name.c_str());
  devices[id].reset();
  devices.erase(id);
}
