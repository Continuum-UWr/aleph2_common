#include "aleph2_manip_kinematics/aleph2_manip_kinematics.h"

#include "std_msgs/Float64.h"
#include "geometry_msgs/TransformStamped.h"

#include "moveit/robot_model/robot_model.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_conversions/kdl_msg.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace aleph2_manip_kinematics
{
    Aleph2ManipKinematics::Aleph2ManipKinematics(bool check_collision)
        : check_collision_(check_collision),
          current_joint_states_(NR_OF_JOINTS+1),
          tf_listener_(tf_buffer_)
    {
        auto urdf_model = std::make_shared<urdf::Model>();
        auto srdf_model = std::make_shared<srdf::Model>();
        std::string robot_description;
        std::string robot_description_semantic;
        std::vector<std::string> tip_frames = {"efector_tip"};

        // Load robot model
        if (!nh_.param("aleph1/robot_description", robot_description, std::string())) {
            ROS_FATAL("Failed to retrieve the robot description from parameter server!");
            ROS_BREAK();
        }
        
        if (!nh_.param("aleph1/robot_description_semantic", robot_description_semantic, std::string())) {
            ROS_FATAL("Failed to retrieve the semantic robot description from parameter server!");
            ROS_BREAK();
        }

        if (!urdf_model->initString(robot_description)) {
            ROS_FATAL("Failed to create urdf model object");
            ROS_BREAK();
        }

        if (!srdf_model->initString(*urdf_model, robot_description_semantic)) {
            ROS_FATAL("Failed to create srdf model object");
            ROS_BREAK();
        }

        if (!kdl_parser::treeFromUrdfModel(*urdf_model, robot_tree_)) {
            ROS_FATAL("Failed to construct the KDL tree");
            ROS_BREAK();
        }

        if (!robot_tree_.getChain("base_link", "efector_tip", manip_chain_)) {
            ROS_FATAL("Failed to get the manip chain from the KDL tree");
            ROS_BREAK();
        }

        auto robot_model = std::make_shared<moveit::core::RobotModel>(
            urdf_model, 
            srdf_model
        );

        // Initialize planning scene
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model);
        robot_state_ = &planning_scene_->getCurrentStateNonConst();

        // Get the inverse kinematics solver plugin
        ik_solver_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>
            ("moveit_core", "kinematics::KinematicsBase");

        try {
            ik_solver_ = ik_solver_loader_->createInstance("aleph_manip_kinematics/IKFastKinematicsPlugin"); 
        }
        catch(pluginlib::PluginlibException& ex) {
            ROS_ERROR("The kinematics plugin failed to load. Error: %s", ex.what());
            ROS_BREAK();
        } 

        // Initialize the IK solver
        if (!ik_solver_->initialize("aleph1/robot_description", "manip", "base_link", tip_frames, 5)){
            ROS_ERROR("The kinematics solver failed to initialize!");
            ROS_BREAK();
        }

        // Initialize the FK solver
        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(manip_chain_);

        // advertise the position command topics
        for (int i = 0; i < NR_OF_JOINTS; ++i) {
            pos_pubs_[i] = nh_.advertise<std_msgs::Float64>(
                "/aleph2/manip/controllers/position/" + CONTROLLERS[i] + "/command", 5);
        }

        fake_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("aleph1/fake_joint_states", 10);
        fake_joint_states_.name = ik_solver_->getJointNames();
    }

    bool Aleph2ManipKinematics::setPose(const geometry_msgs::Pose& pose, KinematicsError& err)
    {
        std::vector<double> solution;
        moveit_msgs::MoveItErrorCodes moveit_error;
        auto callback = std::bind(&Aleph2ManipKinematics::solutionCallback, this, 
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        if (!ik_solver_->searchPositionIK(pose, current_joint_states_, 0.001, solution, callback, moveit_error)) {
            err = KinematicsError::IK_SOLVER_FAILED;
            return false;
        }

        if (check_collision_) {
            const std::vector<std::string>& joint_names = ik_solver_->getJointNames();

            for (int i = 0; i < NR_OF_JOINTS; ++i)
                robot_state_->setJointPositions(joint_names[i], &solution[i]);

            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;

            planning_scene_->checkSelfCollision(collision_request, collision_result);

            if (collision_result.collision){
                err = KinematicsError::SOLUTION_IN_SELF_COLLISION;
                return false;
            }
        }

        for (int i = 0; i < NR_OF_JOINTS; ++i) {
            std_msgs::Float64 angle;
            angle.data = solution[i] * SCALE[i] + OFFSETS[i];
            pos_pubs_[i].publish(angle);
        }

        fake_joint_states_.header.stamp = ros::Time::now();
        fake_joint_states_.position = solution;
        fake_joint_pub_.publish(fake_joint_states_);

        current_joint_states_ = solution;
        current_pose_ = pose;
        return true;
    }

    void Aleph2ManipKinematics::solutionCallback(const geometry_msgs::Pose& ik_pose,
                          const std::vector<double>& ik_solution,
                          moveit_msgs::MoveItErrorCodes& error_code)
    {
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
        double angle = rot_quat.getAngle();

        // Check if the angle is tolerable
        if (angle > ANGLE_TOLERANCE) {
            error_code.val = error_code.FAILURE;
            return;
        }

        // std::cout << "ik_pose: " << std::endl << ik_pose << std::endl;
        // std::cout << "fk_pose: " << std::endl << fk_pose << std::endl;
        // std::cout << "angle: " << angle << std::endl;

        error_code.val = error_code.SUCCESS;
    }

    bool Aleph2ManipKinematics::getCurrentPose(geometry_msgs::Pose &pose)
    {
        geometry_msgs::TransformStamped transform;

        try {
            transform = tf_buffer_.lookupTransform("base_link", "efector_tip", ros::Time(0));
            std::cout << transform << std::endl;
        } catch (tf2::LookupException &ex) {
            return false;
        }

        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z;
        pose.orientation = transform.transform.rotation;

        return true;
    }

    bool Aleph2ManipKinematics::getFKPose(const std::vector<double>& joint_pos, geometry_msgs::Pose& pose)
    {
        KDL::JntArray joint_pos_kdl(5);
        for (int i = 0; i < 5; ++i)
            joint_pos_kdl(i) = joint_pos[i];

        KDL::Frame pose_kdl;
        if (fk_solver_->JntToCart(joint_pos_kdl, pose_kdl) < 0) {
            ROS_FATAL("FK solver failed to find the position of the end-effector");
            return false;
        }

        tf::poseKDLToMsg(pose_kdl, pose);
        return true;
    }
}