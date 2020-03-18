#ifndef ALEPH2_MANIP_KINEMATICS_H
#define ALEPH2_MANIP_KINEMATICS_H

#include "ros/ros.h"

#include "pluginlib/class_loader.h"
#include "moveit/kinematics_base/kinematics_base.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "urdf/model.h"
#include "srdfdom/model.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"

#include "kdl/tree.hpp"
#include "kdl/chainfksolver.hpp"

#include "aleph2_manip/KinematicsConfig.h"


namespace aleph2_manip_kinematics
{
    const int NR_OF_JOINTS = 4;
    const std::string CONTROLLERS[] = {"base", "shoulder", "elbow", "wrist_tilt"};
    const double OFFSETS[] = {0.0, -1.57079632679, 0.0, 0.0};
    const double SCALE[] = {1.0, 1.0, -1.0, 1.0};
    const double ANGLE_TOLERANCE = 0.001;

    enum class KinematicsError
    {
        IK_SOLVER_FAILED,
        SOLUTION_IN_SELF_COLLISION,
        FK_SOLVER_FAILED,
        INVALID_GROUP_STATE
    };

    class Aleph2ManipKinematics
    {
    public:
        /**
         * Load all the necessary resources
         * 
         * @param check_collision if set to true, self-collision checking will be enabled 
         * (default true)
         */
        Aleph2ManipKinematics(const std::string &group_name = "manip");

        /**
         * @brief Set the pose of the end-effector
         * @param pose The pose to move the end-effector to
         * @param result_pose The actual pose of the end-effector returned
         * by the forward kinematics solver. If the function failed, it is the 
         * last correct pose
         * @param err An error code that encodes the reason for failure
         * @return true, if the position was set successfully, false otherwise
         */
        bool setPose(const geometry_msgs::Pose &pose, 
                     geometry_msgs::Pose &result_pose,
                     KinematicsError &err);
        
        /**
         * @brief Set the pose of the end-effector
         * @param position The position to move the end-effector to
         * @param pitch The pitch angle of the end-effector pose
         * @param result_pose The actual pose of the end-effector returned
         * by the forward kinematics solver. If the function failed, it is the 
         * last correct pose
         * @param err An error code that encodes the reason for failure
         * @return true, if the position was set successfully, false otherwise
         */
        bool setPose(const geometry_msgs::Point &position, 
                     const double &pitch, 
                     geometry_msgs::Pose &result_pose,
                     KinematicsError &err);

        /**
         * @brief Set the pose of the end-effector
         * @param group_state The name of the group state defined in the SRDF model
         * @param result_pose The actual pose of the end-effector returned
         * by the forward kinematics solver. If the function failed, it is the 
         * last correct pose
         * @param err An error code that encodes the reason for failure
         * @return true, if the position was set successfully, false otherwise
         */
        bool setPose(const std::string &group_state, 
                     geometry_msgs::Pose &result_pose,
                     KinematicsError &err);
        
        /**
         * @brief Get the current pose of the end-effector tip published on TF
         * @param pose The pose of the end-effector tip
         * @return true, if the position was retrieved successfully, false otherwise
         */
        bool getCurrentPose(geometry_msgs::Pose &pose);

        void setConfig(aleph2_manip::KinematicsConfig &config) { config_ = config; }

    private:

        void solutionCallback(const geometry_msgs::Pose &ik_pose,
                              const std::vector<double> &ik_solution,
                              moveit_msgs::MoveItErrorCodes &error_code);

        bool getFKPose(const std::vector<double> &joint_pos, geometry_msgs::Pose &pose);


        // configuration
        aleph2_manip::KinematicsConfig config_;

        // solvers
        std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> ik_solver_loader_;
        boost::shared_ptr<kinematics::KinematicsBase> ik_solver_;
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
        std::shared_ptr<KDL::ChainFkSolverPos> fk_solver_;

        // memory
        const std::string group_name_;
        const std::vector<std::string> *joint_names_;
        int joints_nr_;
        std::shared_ptr<urdf::Model> urdf_model_;
        std::shared_ptr<srdf::Model> srdf_model_;
        robot_state::RobotState *robot_state_;
        KDL::Tree robot_tree_;
        KDL::Chain manip_chain_;
        geometry_msgs::Pose current_pose_;
        std::vector<double> current_joint_states_;
        sensor_msgs::JointState fake_joint_states_;

        // ROS stuff
        ros::NodeHandle nh_;
        ros::Publisher pos_pubs_[NR_OF_JOINTS];
        ros::Publisher fake_joint_pub_;

        // TF stuff
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        geometry_msgs::TransformStamped manip_transform_;
    };

    double get_pitch(const geometry_msgs::Pose &pose);
}


#endif