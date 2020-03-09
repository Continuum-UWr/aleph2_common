#include "aleph2_manip_kinematics/aleph2_manip_kinematics.h"
#include "aleph2_manip/KinematicsCommand.h"

#include "std_srvs/Empty.h"
#include "geometry_msgs/PoseStamped.h"

geometry_msgs::Pose pose;
aleph2_manip::KinematicsCommand cmd;
aleph2_manip_kinematics::Aleph2ManipKinematics *manip_kinematics;
aleph2_manip_kinematics::KinematicsError error;
bool active = false;

void command_callback(const aleph2_manip::KinematicsCommandConstPtr& msg)
{
    cmd = *msg;
}

bool activate_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (!active) {
        if (!manip_kinematics->getCurrentPose(pose)) {
            ROS_ERROR("Failed to retrieve the current pose of the end-effector!");
            return false;
        }
        cmd = aleph2_manip::KinematicsCommand();
        active = true;
        ROS_INFO("Kinematics control activated");
    } else {
        ROS_WARN("Trying to activate Kinematics control when it is already active!");
    }
    return true;
}

bool deactivate_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (active) {
        active = false;
        ROS_INFO("Kinematics control deactivated");
    } else {
        ROS_WARN("Trying to deactivate Kinematics control when it not active!");
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manip_kinematics_control_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int loop_rate;
    geometry_msgs::PoseStamped pose_stamped;

    pnh.param("loop_rate", loop_rate, 30);
    
    manip_kinematics = new aleph2_manip_kinematics::Aleph2ManipKinematics();

    ros::Subscriber cmd_sub = nh.subscribe("aleph2/manip/kinematics/command", 1, command_callback);
    ros::ServiceServer activate_srv = nh.advertiseService("aleph2/manip/kinematics/activate", activate_callback);
    ros::ServiceServer deactivate_srv = nh.advertiseService("aleph2/manip/kinematics/deactivate", deactivate_callback);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("aleph2/manip/kinematics/pose", 10);

    ros::Time last_update = ros::Time::now();
    ros::Rate rate(loop_rate);

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        ros::spinOnce();

        if (active){ 

            geometry_msgs::Pose new_pose = pose;

            float duration = (current_time - last_update).toSec();

            new_pose.position.x += cmd.ee_x_cmd * duration;
            new_pose.position.y += cmd.ee_y_cmd * duration;
            new_pose.position.z += cmd.ee_z_cmd * duration;

            if (!manip_kinematics->setPose(new_pose, error))
            {
                switch(error)
                {
                    case aleph2_manip_kinematics::KinematicsError::IK_SOLVER_FAILED:
                        ROS_DEBUG("IK solver failed to find solution");
                        break;
                    case aleph2_manip_kinematics::KinematicsError::SOLUTION_IN_SELF_COLLISION:
                        ROS_DEBUG("Solution in self collision");
                        break;
                    default:
                        ROS_ERROR_STREAM("Unknown error! Error code: " << static_cast<int>(error));
                }
            }
            else
            {
                pose = new_pose;

                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.header.frame_id = "base_link";
                pose_stamped.pose = pose;
                pose_pub.publish(pose_stamped);
            }

        }

        last_update = current_time;

        if (!rate.sleep()) {
            ROS_WARN_THROTTLE(10, "The Kinematics control node missed its desired loop rate!");
        }
    }

    delete manip_kinematics;
}