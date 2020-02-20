#include "aleph2_manip_kinematics/aleph2_manip_kinematics.h"

#include "std_msgs/Float64.h"
#include "moveit/robot_model/robot_model.h"

namespace aleph2_manip_kinematics
{
    Aleph2ManipKinematics::Aleph2ManipKinematics(bool check_collision)
        : check_collision_(check_collision),
          current_joint_states_(NR_OF_JOINTS)
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

        if (!ik_solver_->initialize("aleph1/robot_description", "manip", "base_link", tip_frames, 5)){
            ROS_ERROR("The kinematics solver failed to initialize!");
            ROS_BREAK();
        }

        // advertise the position command topics
        for (int i = 0; i < NR_OF_JOINTS; ++i) {
            pos_pubs_[i] = nh_.advertise<std_msgs::Float64>(
                "/aleph2/manip/controllers/position/" + CONTROLLERS[i] + "/command", 5);
        }

    }

    bool Aleph2ManipKinematics::setPose(const geometry_msgs::Pose& pose, KinematicsError& err)
    {
        std::vector<double> seed = {-0.00412247, 0.0648549, 1.01141, -1.07626, 3.14159};
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
            angle.data = solution[i] + OFFSETS[i];
            pos_pubs_[i].publish(angle);
        }

        current_joint_states_ = solution;
        current_pose_ = pose;
        return true;
    }

    void Aleph2ManipKinematics::solutionCallback(const geometry_msgs::Pose& ik_pose,
                          const std::vector<double>& ik_solution,
                          moveit_msgs::MoveItErrorCodes& error_code)
    {
        const double shoulder_angle = ik_solution[1] + OFFSETS[1];

        if (shoulder_angle < 0.0) {
            error_code.val = error_code.GOAL_CONSTRAINTS_VIOLATED;
            return;
        }

        error_code.val = error_code.SUCCESS;
    }

    geometry_msgs::Pose Aleph2ManipKinematics::getCurrentPose()
    {
        return current_pose_;
    }
}