#! /usr/bin/env python
#
# Test client for DMP action server.
#
# Author: Mihael Simonic and Rok Pahic
#

#Calculation
from dmp import dmp_ros
from sensor_msgs.msg import JointState
# Numpy
import numpy as np

# ROS stuff
import rospy

# Brings in the SimpleActionClient
import actionlib

# Action messages
from robot_module_msgs.msg import JointDMPAction,JointDMPFeedback,JointDMPGoal



def test_client():

    namespace="/panda1"
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('/DMP_action_server', JointDMPAction)

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
    # Trajectory time
    T = 5
    # Sample time
    D_T = 0.01

    time_vec = np.arange(0, T, D_T)
    traj = np.array([np.sin(time_vec/time_vec[-1] * 2*np.pi),
                             np.cos(time_vec/time_vec[-1] * 2*np.pi),
                             -np.sin(time_vec/time_vec[-1] * 2*np.pi)
                             ]).transpose()
    
    # Put the trajectory in the JointState array
    traj_ros = []
    for i in range(len(time_vec)):
        sample = JointState()
        sample.position = traj[i]
        sample.header.stamp = rospy.Time(time_vec[i])
        # The following line is an necessary evil because rospy.Time does not
        # directly copy the time. Example: rospy.Time(1.16) = rospy.Time[1159999999]
        sample.header.stamp.nsecs = round(sample.header.stamp.nsecs/10000000.0)*10000000.0
        traj_ros.append(sample)

    # Encode the DMP using both libraries
    encoded_dmp_ros = dmp_ros.encode_dmp(trajectory_samples=traj_ros, num_weights=15)



    try:
        # Sends the goal to the action server.
        goal = JointDMPGoal(encoded_dmp_ros)

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
