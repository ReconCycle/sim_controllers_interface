#! /usr/bin/env python
#
# Action server that integrates DMP and sends commands to panda simulator.
#
# Authors: Mihael Simonic and Rok Pahic
#

# Calculations
from dmp import dmp

# Numpy
import numpy as np

# ROS stuff
import rospy
import actionlib

# ROS messages
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
# Action messages
import robot_module_msgs.msg 

# ===================
# Action Server Class
# ===================
class DMPActionInterface(object):

    _feedback = robot_module_msgs.msg.JointMinJerkFeedback()
    _result = robot_module_msgs.msg.JointMinJerkResult()
    _action_name = ''
    _command_pub = None

    _robotjoints = []
    
    # -----------
    # Constructor
    # -----------
    def __init__(self, name):
        """Constructor."""

        # Subscribe to robot state topics
        rospy.Subscriber('joint_states', JointState, self.getrobotjoints_cb,  tcp_nodelay=True)
        
        # Initialize publisher
        self._command_pub = rospy.Publisher('/panda1/gazebo_panda/effort_joint_position_controller/command',Float64MultiArray, queue_size = 1, tcp_nodelay = True)


        # Start the action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                robot_module_msgs.msg.JointDMPAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)

        # Start the action server
        self._as.start()
        
        # Register rospy shutdown signal handler
        # rospy.on_shutdown(self.stop_robot)


    # ---------
    # Callbacks
    # ---------
    def getrobotjoints_cb(self, joints):
        """Callback function for getting robot joint states."""
        # panda_simulator also publishes finger1 and finger2 to joint state 
        self._robotjoints = joints.position[2:9]

    # ------------------------
    # Main robot motion method
    # ------------------------
    def move_robot_to_point(self, pos_desired, vel_desired):

        # Convert to numpy
        q = np.asarray(pos_desired)
        qdot = np.asarray(vel_desired)

        #Construct message
        cmd_msg = Float64MultiArray()
        
        #cmd_msg.names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        #cmd_msg.position = q
        #cmd_msg.velocity = qdot
        #cmd_msg.mode = cmd_msg.POSITION_MODE
        cmd_msg.data = q  

        #Publish
        self._command_pub.publish(cmd_msg)
        
        rospy.logdebug("Next point sent")

    # ------------------------------------------
    # Main action server execution callback
    # ------------------------------------------
    def execute_cb(self, goal):
        """Main action execution callback method."""
        num_joints=7


        print(goal)
        start_joint_pos = goal.DMP.y0
        end_joint_pos = goal.DMP.goal
        motion_duration = goal.DMP.tau
        motion_timestep = 0.01 #goal.DMP.d_t
        rate = rospy.Rate(1/motion_timestep)

        # Check how far you are starting point
        self._robotjoints


        dmp_exe = dmp(num_weights=goal.DMP.N)
        dmp_exe.tau =
        dmp_exe._d_t = 
        dmp_exe.y0 = goal.DMP.y0.positions
        dmp_exe.goal = goal.DMP.y0.positions
        dmp_exe.


        trj=dmp_exe.decode()
        
        # Calculate trajectory points using DMP
        for i in range(num_joints):
            coeffs[i] = polynomial1(start_joint_pos[i], 0.0, 0.0,
                            end_joint_pos[i], 0.0, 0.0,
                            motion_duration)
            
        joint_pos = np.zeros((int(motion_duration / motion_timestep + 1), num_joints))
        joint_vel = np.zeros((int(motion_duration / motion_timestep + 1), num_joints))

        for i in range(num_joints):
            joint_pos[:, i], joint_vel[:, i], _ = polynomial2(coeffs[i], motion_timestep, motion_duration)
        
        
        # Publish info to the console for the user
        rospy.loginfo("[{0}]: Executing joint motion from current joints:\n{1} to joints: \n{2}"
                      " in {3} seconds.".format(self._action_name,
                                                           start_joint_pos,
                                                           end_joint_pos,
                                                           motion_duration))
        
        # Start executing the action
        for i in range(len(joint_pos)):

            # Check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.logwarn('[{0}]: Preempted'.format(self._action_name))
                self._result.fully_succeeded = False
                self._as.set_preempted(self._result)
                return

            # Send command to the robot
            self.move_robot_to_point(joint_pos[i, :], joint_vel[i, :])
            #print(joint_pos[i, :])

            # Get the current pose as feedback
            self._feedback.current_joint_pos = self._robotjoints
            #self._feedback.joint_diff = ...

            # Publish the feedback
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        rospy.loginfo('[{0}]: Fully succeeded'.format(self._action_name))
        self._result.fully_succeeded = True
        self._as.set_succeeded(self._result)
        return


if __name__ == '__main__':

    try:
        rospy.init_node('DMP_action_server')
        server = DMPActionInterface(rospy.get_name())
        
        rospy.loginfo("Action server \'{0}\' started.".format(rospy.get_name()))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
 
