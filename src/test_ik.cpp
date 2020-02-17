#include <vector>

#include "ros/ros.h"

#include "pluginlib/class_loader.h"
#include "moveit/kinematics_base/kinematics_base.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include "srdfdom/model.h"
#include "moveit/robot_model/robot_model.h"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_conversions/kdl_msg.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/collision_detection/collision_common.h"

void solution_callback(const geometry_msgs::Pose& ik_pose,
                       const std::vector<double>& ik_solution,
                       moveit_msgs::MoveItErrorCodes& error_code)
{
    // std::cout << "Got solution" << std::endl;
    // for (const double& angle : ik_solution){
    //     std::cout << angle << " ";
    // }
    // std::cout << std::endl;

    error_code.val = error_code.SUCCESS;
}

const std::string CONTROLLERS[] = {"base", "shoulder", "elbow", "wrist_tilt", "wrist_roll"};
const double OFFSETS[] = {0.0, -1.57079632679, 0.0, 0.0, 0.0};

// const double STATES[] = {0.42, 2.14, 2.22, 0.0, 0.0};
const double STATES[] = {1.08, 0.98, -1.60, -0.74, 0.0};
// const double STATES[] = {0.5, 0.0, 0.0, 0.0, 0.0};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "test_ik");

    ros::NodeHandle nh;

    KDL::Tree robot_tree;
    KDL::Chain manip_chain;
    auto urdf_model = std::make_shared<urdf::Model>();
    auto srdf_model = std::make_shared<srdf::Model>();
    std::string robot_description;
    std::string robot_description_semantic;

    boost::shared_ptr<kinematics::KinematicsBase> ik_solver;

    std::vector<std::string> tip_frames = {"efector_tip"};


    if (!nh.param("aleph1/robot_description", robot_description, std::string())) {
        ROS_ERROR("Failed to retrieve the robot description from parameter server!");
        ROS_BREAK();
    }

    if (!nh.param("aleph1/robot_description_semantic", robot_description_semantic, std::string())) {
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

    if (!kdl_parser::treeFromUrdfModel(*urdf_model, robot_tree)) {
        ROS_ERROR("Failed to construct kdl tree");
        ROS_BREAK();
    }

    auto robot_model = std::make_shared<moveit::core::RobotModel>(
        urdf_model, 
        srdf_model
    );

    planning_scene::PlanningScene planning_scene(robot_model);




    if (!robot_tree.getChain("base_link", "efector_tip", manip_chain)) {
        ROS_ERROR("Failed to get manip chain!");
        ROS_BREAK();
    }

    KDL::ChainFkSolverPos_recursive fk_solver(manip_chain);

    KDL::JntArray joint_pos(5);
    for (int i = 0; i < 5; ++i)
        joint_pos(i) = 0.0;

    KDL::Frame pos;

    if (fk_solver.JntToCart(joint_pos, pos) < 0) {
        ROS_ERROR("Failed to solve forward kinematics pose!");
        ROS_BREAK();
    }

    geometry_msgs::Pose pose;
    tf::poseKDLToMsg(pos, pose);

    std::cout << pose << std::endl;


    pluginlib::ClassLoader<kinematics::KinematicsBase> kinematicsBase_loader
        ("moveit_core", "kinematics::KinematicsBase");

    try {
        ik_solver = kinematicsBase_loader.createInstance("aleph_manip_kinematics/IKFastKinematicsPlugin"); 
    }
    catch(pluginlib::PluginlibException& ex) {
        ROS_ERROR("The kinematics plugin failed to load. Error: %s", ex.what());
        ROS_BREAK();
    } 

    if (!ik_solver->initialize("aleph1/robot_description", "manip", "base_link", tip_frames, 5)){
        ROS_ERROR("The kinematics solver failed to initialize!");
        ROS_BREAK();
    }

    robot_state::RobotState& robot_state = planning_scene.getCurrentStateNonConst();
    const std::vector<std::string>& joint_names = ik_solver->getJointNames();

    for (int i = 0; i < joint_names.size(); ++i)
        robot_state.setJointPositions(joint_names[i], &STATES[i]);

    collision_detection::CollisionRequest collision_request;
    // collision_request.group_name = parameters.move_group_name;
    // collision_request.distance = true;
    collision_detection::CollisionResult collision_result;
    collision_result.clear();

    planning_scene.checkSelfCollision(collision_request, collision_result);

    std::cout << collision_result.collision << "\n";

    return 0;



    // geometry_msgs::Pose desiredPose;
    // desiredPose.position.x = 0.5;
    // desiredPose.position.z = 0.5;
    // desiredPose.orientation.w = 1;

    std::vector<double> seed = {-0.00412247, 0.0648549, 1.01141, -1.07626, 3.14159};

    std::vector<double> solution;
    moveit_msgs::MoveItErrorCodes error;

    // std::vector<ros::Publisher> pos_pubs(5);
    // for (int i = 0; i < 5; ++i) {
    //     pos_pubs[i] = nh.advertise<std_msgs::Float64>(
    //         "/aleph2/manip/controllers/position/" + CONTROLLERS[i] + "/command", 5);
    // }

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("aleph1/joint_states", 10);
    sensor_msgs::JointState states;
    states.name = ik_solver->getJointNames();

    ros::Rate rate(10);
    while (ros::ok())
    {
        if (ik_solver->searchPositionIK(pose, seed, 0.001, solution, solution_callback, error)) {
            // for (int i = 0; i < 5; ++i){
            //     std_msgs::Float64 angle;
            //     angle.data = solution[i] + OFFSETS[i];
            //     pos_pubs[i].publish(angle);
            // }
            states.position = solution;
            states.header.stamp = ros::Time::now();
            joint_pub.publish(states);
        } else {
            ROS_ERROR("No inverse kinematics solution found!");
        }

        rate.sleep();
    }

}
