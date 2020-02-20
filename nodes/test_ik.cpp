#include "aleph2_manip_kinematics/aleph2_manip_kinematics.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ik");

    aleph2_manip_kinematics::Aleph2ManipKinematics kinematics;
    aleph2_manip_kinematics::KinematicsError error;

    ros::Duration(0.5).sleep();

    geometry_msgs::Pose pose;
    pose.position.x = 0.6;
    pose.position.z = 0.4;
    pose.orientation.w = 1.0;

    if (!kinematics.setPose(pose, error))
    {
        switch(error)
        {
            case aleph2_manip_kinematics::KinematicsError::IK_SOLVER_FAILED:
                ROS_ERROR("IK solver failed to find solution");
                break;
            case aleph2_manip_kinematics::KinematicsError::SOLUTION_IN_SELF_COLLISION:
                ROS_ERROR("Solution in self collision");
                break;
            default:
                ROS_ERROR_STREAM("Unknown error! Error code: " << static_cast<int>(error));
        }
    }
    else
    {
        std::cout << "Pose set successfully!\n";
    }
    

    ros::spin();
}