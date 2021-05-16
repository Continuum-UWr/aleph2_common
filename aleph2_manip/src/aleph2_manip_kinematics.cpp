#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>

#include <kdl_conversions/kdl_msg.h>
#include <moveit/robot_model/robot_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <aleph2_manip_kinematics/aleph2_manip_kinematics.h>

namespace aleph2_manip_kinematics {

Aleph2ManipKinematics::Aleph2ManipKinematics(const std::string &group_name)
    : group_name_(group_name), tf_listener_(tf_buffer_) {
  urdf_model_ = std::make_shared<urdf::Model>();
  srdf_model_ = std::make_shared<srdf::Model>();
  std::string robot_description;
  std::string robot_description_semantic;
  std::vector<std::string> tip_frames = {"efector_tip"};

  // Load robot model
  if (!nh_.param("aleph1/robot_description", robot_description,
                 std::string())) {
    ROS_FATAL(
        "Failed to retrieve the robot description from parameter server!");
    ROS_BREAK();
  }

  if (!nh_.param("aleph1/robot_description_semantic",
                 robot_description_semantic, std::string())) {
    ROS_FATAL(
        "Failed to retrieve the semantic robot description from parameter "
        "server!");
    ROS_BREAK();
  }

  if (!urdf_model_->initString(robot_description)) {
    ROS_FATAL("Failed to create urdf model object");
    ROS_BREAK();
  }

  if (!srdf_model_->initString(*urdf_model_, robot_description_semantic)) {
    ROS_FATAL("Failed to create srdf model object");
    ROS_BREAK();
  }

  auto robot_model =
      std::make_shared<moveit::core::RobotModel>(urdf_model_, srdf_model_);

  // Get the KDL robot tree and chain for the manipulator
  if (!kdl_parser::treeFromUrdfModel(*urdf_model_, robot_tree_)) {
    ROS_FATAL("Failed to construct the KDL tree");
    ROS_BREAK();
  }

  if (!robot_tree_.getChain("base_link", "efector_tip", manip_chain_)) {
    ROS_FATAL("Failed to get the manip chain from the KDL tree");
    ROS_BREAK();
  }

  // Initialize the planning scene
  planning_scene_ =
      std::make_shared<planning_scene::PlanningScene>(robot_model);
  robot_state_ = &planning_scene_->getCurrentStateNonConst();

  // Get the inverse kinematics solver plugin
  ik_solver_loader_ =
      std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
          "moveit_core", "kinematics::KinematicsBase");

  try {
    ik_solver_ = ik_solver_loader_->createInstance(
        "aleph_manip_kinematics/IKFastKinematicsPlugin");
  } catch (pluginlib::PluginlibException &ex) {
    ROS_FATAL("The kinematics plugin failed to load. Error: %s", ex.what());
    ROS_BREAK();
  }

  // Initialize the IK solver
  if (!ik_solver_->initialize("aleph1/robot_description", group_name_,
                              "base_link", tip_frames, 5)) {
    ROS_FATAL("The kinematics solver failed to initialize!");
    ROS_BREAK();
  }

  // Get information about joints
  joint_names_ = &ik_solver_->getJointNames();
  joints_nr_ = joint_names_->size();
  current_joint_states_.resize(joints_nr_);

  // Initialize the FK solver
  fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(manip_chain_);

  // advertise the position command topics
  for (int i = 0; i < NR_OF_JOINTS; ++i) {
    pos_pubs_[i] = nh_.advertise<std_msgs::Float64>(
        "/aleph2/manip/controllers/position/" + CONTROLLERS[i] + "/command", 5);
  }

  // advertise the fake joint states topic
  fake_joint_pub_ =
      nh_.advertise<sensor_msgs::JointState>("aleph1/fake_joint_states", 10);
  fake_joint_states_.name = *joint_names_;

  // Get the transform to manip's base link
  try {
    manip_transform_ = tf_buffer_.lookupTransform(
        "manip_base", "base_link", ros::Time(0), ros::Duration(10.0));
  } catch (tf2::LookupException &ex) {
    ROS_FATAL_STREAM(
        "Failed to get the transform to manip_base link: " << ex.what());
    ROS_BREAK();
  }
}

bool Aleph2ManipKinematics::setPose(const geometry_msgs::Pose &pose,
                                    geometry_msgs::Pose &result_pose,
                                    KinematicsError &err) {
  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes moveit_error;
  auto callback = std::bind(&Aleph2ManipKinematics::solutionCallback, this,
                            std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3);

  // Search for the inverse kinematics solution
  if (!ik_solver_->searchPositionIK(pose, current_joint_states_, 0.001,
                                    solution, callback, moveit_error)) {
    err = KinematicsError::IK_SOLVER_FAILED;
    result_pose = current_pose_;
    return false;
  }

  // Check for self-collision
  if (config_.check_collision) {
    const std::vector<std::string> &joint_names = ik_solver_->getJointNames();

    for (int i = 0; i < NR_OF_JOINTS; ++i)
      robot_state_->setJointPositions(joint_names[i], &solution[i]);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    planning_scene_->checkSelfCollision(collision_request, collision_result);

    if (collision_result.collision) {
      err = KinematicsError::SOLUTION_IN_SELF_COLLISION;
      result_pose = current_pose_;
      return false;
    }
  }

  // Get the actual pose of the end-effector for the solution
  if (!getFKPose(solution, current_pose_)) {
    err = KinematicsError::FK_SOLVER_FAILED;
    return false;
  }

  current_joint_states_ = solution;
  result_pose = current_pose_;

  // Publish joint position commands
  for (int i = 0; i < NR_OF_JOINTS; ++i) {
    std_msgs::Float64 angle;
    angle.data = solution[i] * SCALE[i] + OFFSETS[i];
    pos_pubs_[i].publish(angle);
  }

  // Publish fake joint states
  fake_joint_states_.header.stamp = ros::Time::now();
  fake_joint_states_.position = solution;
  fake_joint_pub_.publish(fake_joint_states_);

  return true;
}

bool Aleph2ManipKinematics::setPose(const geometry_msgs::Point &position,
                                    const double &pitch,
                                    geometry_msgs::Pose &result_pose,
                                    KinematicsError &err) {
  geometry_msgs::Pose pose;
  pose.position = position;

  // Get the position in manip's base frame
  geometry_msgs::Point manip_point;
  tf2::doTransform(pose.position, manip_point, manip_transform_);

  // Calculate the Z angle
  tf2::Vector3 z_axis(0.0, 0.0, 1.0);
  double z_angle = atan2(manip_point.y, manip_point.x);

  // Get the correct rotation
  tf2::Matrix3x3 res_mat;
  res_mat.setEulerYPR(z_angle, pitch, 0.0);
  tf2::Quaternion res_quat;
  res_mat.getRotation(res_quat);

  // Some magic with quaternions to get the correct orientation
  // tf2::Quaternion z_quat, y_quat, temp_quat, res_quat;
  // z_quat.setRotation(z_axis, z_angle);
  // temp_quat = z_quat * tf2::Quaternion(0.0, 1.0, 0.0, 0.0) *
  // z_quat.inverse(); tf2::Vector3 y_axis = temp_quat.getAxis();
  // y_quat.setRotation(y_axis, pitch);
  // res_quat = y_quat * z_quat;

  pose.orientation = tf2::toMsg(res_quat);

  return setPose(pose, result_pose, err);
}

bool Aleph2ManipKinematics::setPose(const std::string &group_state,
                                    geometry_msgs::Pose &result_pose,
                                    KinematicsError &err) {
  for (auto state : srdf_model_->getGroupStates()) {
    if (state.group_ == group_name_ && state.name_ == group_state) {
      // state.joint_values_
      KDL::JntArray joint_pos_kdl(5);

      for (int i = 0; i < joints_nr_; ++i) {
        const std::string &joint_name = (*joint_names_)[i];
        try {
          joint_pos_kdl(i) = state.joint_values_.at(joint_name)[0];
        } catch (std::out_of_range &ex) {
          ROS_WARN_STREAM("No '" << joint_name << "' joint in group state '"
                                 << state.name_ << "'");
        }
      }

      KDL::Frame pose_kdl;
      if (fk_solver_->JntToCart(joint_pos_kdl, pose_kdl) < 0) {
        err = KinematicsError::FK_SOLVER_FAILED;
        return false;
      }

      geometry_msgs::Pose pose;
      tf::poseKDLToMsg(pose_kdl, pose);

      return setPose(pose, result_pose, err);
    }
  }

  err = KinematicsError::INVALID_GROUP_STATE;
  return false;
}

void Aleph2ManipKinematics::solutionCallback(
    const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_solution,
    moveit_msgs::MoveItErrorCodes &error_code) {
  const double elbow_angle = ik_solution[2] + OFFSETS[2];

  // Check the angle on the elbow joint
  if (elbow_angle > 0.0) {
    error_code.val = error_code.FAILURE;
    return;
  }

  // Get the FK pose
  geometry_msgs::Pose fk_pose;
  if (!getFKPose(ik_solution, fk_pose)) {
    error_code.val = error_code.FAILURE;
    return;
  }

  // Get the angle between FK and IK poses
  tf2::Quaternion fk_quat, ik_quat, rot_quat;
  tf2::convert(fk_pose.orientation, fk_quat);
  tf2::convert(ik_pose.orientation, ik_quat);
  rot_quat = fk_quat * ik_quat.inverse();
  double angle = rot_quat.getAngleShortestPath();

  // Check if the angle is tolerable
  if (angle > ANGLE_TOLERANCE) {
    error_code.val = error_code.FAILURE;
    return;
  }

  error_code.val = error_code.SUCCESS;
}

bool Aleph2ManipKinematics::getCurrentPose(geometry_msgs::Pose &pose) {
  geometry_msgs::TransformStamped transform;

  try {
    transform =
        tf_buffer_.lookupTransform("base_link", "efector_tip", ros::Time(0));
    // std::cout << transform << std::endl;
  } catch (tf2::LookupException &ex) {
    return false;
  }

  pose.position.x = transform.transform.translation.x;
  pose.position.y = transform.transform.translation.y;
  pose.position.z = transform.transform.translation.z;
  pose.orientation = transform.transform.rotation;

  return true;
}

bool Aleph2ManipKinematics::getFKPose(const std::vector<double> &joint_pos,
                                      geometry_msgs::Pose &pose) {
  KDL::JntArray joint_pos_kdl(5);
  for (int i = 0; i < 5; ++i) joint_pos_kdl(i) = joint_pos[i];

  KDL::Frame pose_kdl;
  if (fk_solver_->JntToCart(joint_pos_kdl, pose_kdl) < 0) {
    ROS_FATAL("FK solver failed to find the position of the end-effector");
    return false;
  }

  tf::poseKDLToMsg(pose_kdl, pose);
  return true;
}

double get_pitch(const geometry_msgs::Pose &pose) {
  tf2::Quaternion rot_quat;
  tf2::convert(pose.orientation, rot_quat);

  tf2::Matrix3x3 rot_mat(rot_quat);

  double roll, pitch, yaw;
  rot_mat.getEulerYPR(yaw, pitch, roll);

  return pitch;
}

}  // namespace aleph2_manip_kinematics