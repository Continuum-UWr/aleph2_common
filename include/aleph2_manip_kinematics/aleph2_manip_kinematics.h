#ifndef ALEPH2_MANIP_KINEMATICS_H
#define ALEPH2_MANIP_KINEMATICS_H

#include "ros/ros.h"

#include "pluginlib/class_loader.h"
#include "moveit/kinematics_base/kinematics_base.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "urdf/model.h"
#include "srdfdom/model.h"

#include "geometry_msgs/Pose.h"

// #include "sensor_msgs/JointState.h"
// #include "kdl_parser/kdl_parser.hpp"
// #include "kdl/tree.hpp"
// #include "kdl/chain.hpp"
// #include "kdl/chainfksolverpos_recursive.hpp"
// #include "kdl_conversions/kdl_msg.h"

namespace aleph2_manip_kinematics
{
    const int NR_OF_JOINTS = 5;
    const std::string CONTROLLERS[] = {"base", "shoulder", "elbow", "wrist_tilt", "wrist_roll"};
    const double OFFSETS[] = {0.0, -1.57079632679, 0.0, 0.0, 0.0};

    class Aleph2ManipKinematics
    {
    public:
        /**
         * Load all the necessary resources
         * 
         * @param check_collision if set to true, self-collision checking will be enabled 
         * (default true)
         */
        Aleph2ManipKinematics(bool check_collision = true);

        /**
         * set the pose of the end-effector
         * 
         * @param pose The pose to move the end-effector to
         * @return true, if the position is reachable, false otherwise
         */
        bool setPose(geometry_msgs::Pose pose);
        
        /**
         * get the current end-effector pose
         * 
         * @return the current pose of the end-effector
         */
        geometry_msgs::Pose getCurrentPose();
    
    private:
        ros::NodeHandle nh_;
        bool check_collision_;
        std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> ik_solver_loader_;
        boost::shared_ptr<kinematics::KinematicsBase> ik_solver_;
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
        robot_state::RobotState* robot_state_;

        ros::Publisher pos_pubs_[NR_OF_JOINTS];
    };
}


#endif