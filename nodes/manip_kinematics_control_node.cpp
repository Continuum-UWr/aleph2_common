#include "aleph2_manip_kinematics/aleph2_manip_kinematics.h"
#include "aleph2_manip/KinematicsCommand.h"

geometry_msgs::Pose pose;
aleph2_manip::KinematicsCommand cmd;

void command_callback(const aleph2_manip::KinematicsCommandConstPtr& msg)
{
    cmd = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manip_kinematics_control_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int loop_rate;

    pnh.param("loop_rate", loop_rate, 10);

    ros::Subscriber cmd_sub = nh.subscribe("aleph2/manip/kinematics/command", 1, command_callback);

    aleph2_manip_kinematics::Aleph2ManipKinematics kinematics;
    aleph2_manip_kinematics::KinematicsError error;

    // ros::Duration(0.5).sleep();

    pose.position.x = 0.8;
    pose.position.z = 0.4;
    pose.orientation.w = 1.0;

    ros::Time last_update = ros::Time::now();
    ros::Rate rate(loop_rate);

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        ros::spinOnce();

        geometry_msgs::Pose new_pose = pose;

        float duration = (current_time - last_update).toSec();

        new_pose.position.x += cmd.ee_x_cmd * duration;
        new_pose.position.z += cmd.ee_z_cmd * duration;

        if (!kinematics.setPose(new_pose, error))
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
            pose = new_pose;
            std::cout << "Pose set successfully!\n";
        }

        last_update = current_time;
        rate.sleep();
    }

    ros::spin();
}