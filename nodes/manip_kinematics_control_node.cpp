#include "aleph2_manip_kinematics/aleph2_manip_kinematics.h"
#include "aleph2_manip/KinematicsCommand.h"

#include "std_srvs/Empty.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

geometry_msgs::Pose ee_pose;
geometry_msgs::Point ee_position;
double ee_pitch;
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
        if (!manip_kinematics->getCurrentPose(ee_pose) || !manip_kinematics->getCurrentPitch(ee_pitch)) {
            ROS_ERROR("Failed to retrieve the current pose of the end-effector!");
            return false;
        }
        ee_position = ee_pose.position;
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
    geometry_msgs::PoseStamped pose_msg;

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

            double duration = (current_time - last_update).toSec();

            geometry_msgs::Point new_ee_position = ee_position;
            double new_ee_pitch = ee_pitch;

            // Apply linear translation
            new_ee_position.x += cmd.ee_x_cmd * duration;
            new_ee_position.y += cmd.ee_y_cmd * duration;
            new_ee_position.z += cmd.ee_z_cmd * duration;

            // Apply pitch rotation
            new_ee_pitch += cmd.ee_pitch_cmd * duration;

            if (!manip_kinematics->setPose(new_ee_position, new_ee_pitch, ee_pose, error))
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
                ee_position = new_ee_position;
                ee_pitch = new_ee_pitch;

                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.frame_id = "base_link";
                pose_msg.pose = ee_pose;
                pose_pub.publish(pose_msg);
            }
        }

        last_update = current_time;

        if (!rate.sleep()) {
            ROS_WARN_THROTTLE(10, "The Kinematics control node missed its desired loop rate!");
        }
    }

    delete manip_kinematics;
}