#include "aleph2_manip_kinematics/aleph2_manip_kinematics.h"

#include "std_msgs/Float64.h"
#include "moveit/robot_model/robot_model.h"

namespace aleph2_manip_kinematics
{
    Aleph2ManipKinematics::Aleph2ManipKinematics(bool check_collision)
        : check_collision_(check_collision)
    {
        auto urdf_model = std::make_shared<urdf::Model>();
        auto srdf_model = std::make_shared<srdf::Model>();
        std::string robot_description;
        std::string robot_description_semantic;
        std::vector<std::string> tip_frames = {"efector_tip"};
        // KDL::Tree robot_tree;

        // Load robot model
        if (!nh_.param("aleph1/robot_description", robot_description, std::string())) {
            ROS_ERROR("Failed to retrieve the robot description from parameter server!");
            ROS_BREAK();
        }
        
        if (!nh_.param("aleph1/robot_description_semantic", robot_description_semantic, std::string())) {
            ROS_ERROR("Failed to retrieve the semantic robot description from parameter server!");
            ROS_BREAK();
        }

        if (!urdf_model->initString(robot_description)) {
            ROS_ERROR("Failed to create urdf model object");
            ROS_BREAK();
        }

        if (!srdf_model->initString(*urdf_model, robot_description_semantic)) {
            ROS_ERROR("Failed to create srdf model object");
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

        if (!ik_solver->initialize("aleph1/robot_description", "manip", "base_link", tip_frames, 5)){
            ROS_ERROR("The kinematics solver failed to initialize!");
            ROS_BREAK();
        }

        // advertise the position command topics
        for (int i = 0; i < NR_OF_JOINTS; ++i) {
            pos_pubs_[i] = nh_.advertise<std_msgs::Float64>(
                "/aleph2/manip/controllers/position/" + CONTROLLERS[i] + "/command", 5);
        }

    }

    bool Aleph2ManipKinematics::setPose(geometry_msgs::Pose pose)
    {
        return true;
    }

    geometry_msgs::Pose Aleph2ManipKinematics::getCurrentPose()
    {
        return geometry_msgs::Pose();
    }
}