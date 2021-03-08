#! /usr/bin/env python
#
# Test client for joint_min_jerk action server.
#
# Author: Mihael Simonic
#

# Numpy
import numpy as np

# ROS stuff
import rospy

# Brings in the SimpleActionClient
import actionlib

# Action messages
from robot_module_msgs.msg import JointMinJerkAction,JointMinJerkFeedback,JointMinJerkGoal

def test_client():

    namespace="/panda1"
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient(namespace+'/joint_min_jerk_action_server', JointMinJerkAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Parameters
    q0=[-0.04, -0.20, 0.20, -2.00, 0.018, 1.83, 1]
    q1=[0.04, 0.33, 0.01, -1.48, 0.02, 1.91, 0.74]
    motion_duration=2
    motion_timestep=0.005

    # Wait for confirmation
    #raw_input("Go to {0}!".format(q0))

    try:
        # Sends the goal to the action server.
        goal = JointMinJerkGoal(q0,motion_duration,motion_timestep)
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        client.wait_for_result()
        # Prints out the result of executing the action
        result = client.get_result() 
        print(result)

        return True
    except Exception as e:
        rospy.logerr("Exception:\n{0}".format(e))
        return False

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('joint_min_jerk_action_test_client')
        test_client()

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
