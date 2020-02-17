#include "aleph2_manip_kinematics/aleph2_manip_kinematics.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ik");

    aleph2_manip_kinematics::Aleph2ManipKinematics kinematics;

    std::cout << kinematics.setPose(geometry_msgs::Pose()) << "\n";
}