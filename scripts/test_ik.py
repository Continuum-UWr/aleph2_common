#!/usr/bin/env python

import rospy

from trac_ik_python.trac_ik import IK

from std_msgs.msg import Float64


CONTROLLERS = ("base", "shoulder", "elbow", "wrist_tilt", "wrist_roll")


rospy.init_node("test_ik")

ik_solver = IK("base_link", "manip_effector_tip_frame")

joints = ik_solver.joint_names
joint_pub = []

for c in CONTROLLERS:
    pub = rospy.Publisher(
        "/aleph2/manip/controllers/position/" + c + "/command",
        Float64,
        queue_size=1
    )
    joint_pub.append(pub)

qinit = [0.] * ik_solver.number_of_joints

for i in range(100):
    start = rospy.get_rostime()
    print(ik_solver.get_ik(qinit, 0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.001, 0.001, 0.001, 0.1, 0.1, 0.1))
    end = rospy.get_rostime()

    print((end-start).to_sec())

rospy.sleep(1)