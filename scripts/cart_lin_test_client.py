#! /usr/bin/env python
#
# Test client for P2P motion action server.
# Based on: https://repo.ijs.si/reconcell/robot_module/-/blob/master/py/cart_lin_task_action_test_client.py
# Authors: Barry Ridge, Mihael Simonic
#

# Numpy
import numpy as np

# ROS stuff
import rospy

# Brings in the SimpleActionClient
import actionlib

# Ros messages
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

# Action messages
import robot_module_msgs.msg

# TF
import tf2_ros
import tf2_geometry_msgs


def cartesian_motion_test_client():

    # Create the TF2 buffer and listener to acquire current robot end effector pose
    tf2_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)

    # Give TF2 Buffer time to start working
    rospy.sleep(0.2)


    # The Z offset to travel
    z_offset = -0.10

    # Acquire current position and construct a target that is some cm from it
    t_pose = tf2_buffer.lookup_transform('base', 'panda_link8', rospy.Time())
    offset_z = PoseStamped(pose=Pose(position=Point(0.0, 0.0, z_offset), orientation=Quaternion(0, 0, 0, 1)))
    t_pose_target = tf2_geometry_msgs.do_transform_pose(offset_z, t_pose)

    # Creates the SimpleActionClient, passing the type of the action
    # (CartLinTaskAction) to the constructor.
    client = actionlib.SimpleActionClient('/cart_lin_action_server', robot_module_msgs.msg.CartLinTaskAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal_cart_trap_down = robot_module_msgs.msg.CartLinTaskGoal([t_pose_target.pose], 1, None)
    goal_cart_trap_back = robot_module_msgs.msg.CartLinTaskGoal(
        [Pose(position=t_pose.transform.translation, orientation=t_pose.transform.rotation)], 1, None)

    rospy.loginfo("Testing normal execution")
    # Sends the goal to the action server.
    client.send_goal(goal_cart_trap_down)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    
    # Go back where you started
    client.send_goal(goal_cart_trap_back)
    client.wait_for_result()

    rospy.loginfo("Testing preemption")
    client.send_goal(goal_cart_trap_down)
    rospy.sleep(0.5)
    client.cancel_all_goals()
    client.wait_for_result()

    rospy.loginfo("Preemption happened, going back.")
    # Go back where you started
    client.send_goal(goal_cart_trap_back)
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('cart_lin_action_test_client')
        result = cartesian_motion_test_client()
        rospy.loginfo("Result: \n{0}\n{1}".format(result.final_pose, result.fully_succeeded))
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
