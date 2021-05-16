#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <aleph2_manip/KinematicsCommand.h>
#include <aleph2_manip_kinematics/aleph2_manip_kinematics.h"

geometry_msgs::Pose ee_pose;
geometry_msgs::Point ee_position;
double ee_pitch;
aleph2_manip::KinematicsCommand cmd;
aleph2_manip_kinematics::Aleph2ManipKinematics* manip_kinematics;
aleph2_manip_kinematics::KinematicsError error;
bool active = false;

void command_callback(const aleph2_manip::KinematicsCommandConstPtr& msg) {
  cmd = *msg;
}

void config_callback(aleph2_manip::KinematicsConfig& config, uint32_t level) {
  manip_kinematics->setConfig(config);
}

bool activate_callback(std_srvs::Empty::Request& request,
                       std_srvs::Empty::Response& response) {
  if (!active) {
    if (!manip_kinematics->getCurrentPose(ee_pose)) {
      ROS_ERROR("Failed to retrieve the current pose of the end-effector!");
      return false;
    }
    ee_position = ee_pose.position;
    ee_pitch = aleph2_manip_kinematics::get_pitch(ee_pose);
    cmd = aleph2_manip::KinematicsCommand();
    active = true;
    ROS_INFO("Kinematics control activated");
  } else {
    ROS_WARN(
        "Trying to activate Kinematics control when it is already active!");
  }
  return true;
}

bool deactivate_callback(std_srvs::Empty::Request& request,
                         std_srvs::Empty::Response& response) {
  if (active) {
    active = false;
    ROS_INFO("Kinematics control deactivated");
  } else {
    ROS_WARN("Trying to deactivate Kinematics control when it not active!");
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "manip_kinematics_control_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int loop_rate;
  geometry_msgs::PoseStamped pose_msg;

  pnh.param("loop_rate", loop_rate, 30);

  manip_kinematics =
      new aleph2_manip_kinematics::Aleph2ManipKinematics("manip");

  ros::Subscriber cmd_sub =
      nh.subscribe("aleph2/manip/kinematics/command", 1, command_callback);
  ros::ServiceServer activate_srv = nh.advertiseService(
      "aleph2/manip/kinematics/activate", activate_callback);
  ros::ServiceServer deactivate_srv = nh.advertiseService(
      "aleph2/manip/kinematics/deactivate", deactivate_callback);

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "aleph2/manip/kinematics/pose", 10);

  dynamic_reconfigure::Server<aleph2_manip::KinematicsConfig> config_server;
  config_server.setCallback(boost::bind(&config_callback, _1, _2));

  ros::Time last_update = ros::Time::now();
  ros::Rate rate(loop_rate);

  while (ros::ok()) {
    ros::Time current_time = ros::Time::now();
    ros::spinOnce();

    if (active) {
      bool success;

      if (cmd.pose_cmd) {
        if (cmd.pose_cmd == cmd.SAFE_POSE)
          success = manip_kinematics->setPose("safe", ee_pose, error);
        else if (cmd.pose_cmd == cmd.ZERO_POSE)
          success = manip_kinematics->setPose("zero", ee_pose, error);

        if (success) {
          ee_position = ee_pose.position;
          ee_pitch = aleph2_manip_kinematics::get_pitch(ee_pose);
        }
      } else {
        double duration = (current_time - last_update).toSec();

        geometry_msgs::Point new_ee_position = ee_position;
        double new_ee_pitch = ee_pitch;

        // Apply linear translation
        new_ee_position.x += cmd.ee_x_cmd * duration;
        new_ee_position.y += cmd.ee_y_cmd * duration;
        new_ee_position.z += cmd.ee_z_cmd * duration;

        // Apply pitch rotation
        new_ee_pitch += cmd.ee_pitch_cmd * duration;

        // Get the translation in tip's frame
        geometry_msgs::Vector3 ee_tip_vector;
        ee_tip_vector.x = cmd.ee_tip_x_cmd * duration;
        ee_tip_vector.y = cmd.ee_tip_y_cmd * duration,
        ee_tip_vector.z = cmd.ee_tip_z_cmd * duration;

        // Rotate the vector
        geometry_msgs::TransformStamped rot_transform;
        rot_transform.transform.rotation = ee_pose.orientation;
        tf2::doTransform(ee_tip_vector, ee_tip_vector, rot_transform);

        // Apply the translation
        new_ee_position.x += ee_tip_vector.x;
        new_ee_position.y += ee_tip_vector.y;
        new_ee_position.z += ee_tip_vector.z;

        success = manip_kinematics->setPose(new_ee_position, new_ee_pitch,
                                            ee_pose, error);

        if (success) {
          ee_position = new_ee_position;
          ee_pitch = new_ee_pitch;
        }
      }

      if (!success) {
        switch (error) {
          case aleph2_manip_kinematics::KinematicsError::IK_SOLVER_FAILED:
            ROS_DEBUG("IK solver failed to find solution");
            break;
          case aleph2_manip_kinematics::KinematicsError::
              SOLUTION_IN_SELF_COLLISION:
            ROS_DEBUG("Solution in self collision");
            break;
          case aleph2_manip_kinematics::KinematicsError::FK_SOLVER_FAILED:
            ROS_ERROR("FK solver failed to find the pose");
            break;
          case aleph2_manip_kinematics::KinematicsError::INVALID_GROUP_STATE:
            ROS_ERROR("Group state not found in SRDF model");
            break;
          default:
            ROS_ERROR_STREAM(
                "Unknown error! Error code: " << static_cast<int>(error));
        }
      }

      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "base_link";
      pose_msg.pose = ee_pose;
      pose_pub.publish(pose_msg);
    }

    last_update = current_time;

    if (!rate.sleep()) {
      ROS_WARN_THROTTLE(
          10, "The Kinematics control node missed its desired loop rate!");
    }
  }

  delete manip_kinematics;
}