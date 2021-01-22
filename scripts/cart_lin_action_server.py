#! /usr/bin/env python
#
# Action server that sends commands to cartesian impedance controller from ijs_controllers.
#
# Authors: Mihael Simonic
#

# Calculations
from minimum_jerk import polynomial1, polynomial2
from minimum_jerk_slerp import min_jerk_slerp

# Numpy
import numpy as np

# ROS stuff
import rospy
import actionlib
import tf2_ros

# ROS messages
#from franka_msgs.msg import FrankaState

# Action messages
import robot_module_msgs.msg 

# ===================
# Action Server Class
# ===================
class CartLinActionInterface(object):

    _feedback = robot_module_msgs.msg.CartLinTaskFeedback()
    _result = robot_module_msgs.msg.CartLinTaskResult()
    _action_name = ''
    _command_pub = None

    _robot_pos = []
    _robot_ori = []

    tfBuffer = None
    listener = None

    # -----------
    # Constructor
    # -----------
    def __init__(self, name):
        """Constructor."""

        # Get pose of the tip in base frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Initialize publisher
        self._command_pub = rospy.Publisher("/cartesian_impedance_controller/command", robot_module_msgs.msg.CartesianCommand, queue_size=1)
        
        # Start the action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                robot_module_msgs.msg.CartLinTaskAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)

        # Start the action server
        self._as.start()


    # ------------------------
    # Main robot motion method
    # ------------------------
    def move_robot_to_point(self, pos_desired, vel_desired):

        # Convert to numpy
        q = np.asarray(pos_desired)
        qdot = np.asarray(vel_desired)

        #Construct message
        cmd_msg = robot_module_msgs.msg.JointCommand()
        cmd_msg.pos = q
        cmd_msg.vel = qdot
        cmd_msg.impedance.n = 7 
        # High stiffness values 
        cmd_msg.impedance.k = [1200,1200,1200,600,250,250,50]
        cmd_msg.impedance.d = [25,25,25,25,10,10,10]
        #Controller crashes if these values are not set
        cmd_msg.trq = [0,0,0,0,0,0,0]
        cmd_msg.acc = [0,0,0,0,0,0,0]

        
        #Publish
        self._command_pub.publish(cmd_msg)
        
        rospy.logdebug("Next point sent")

    # ------------------------------------------
    # Main action server execution callback
    # ------------------------------------------
    def execute_cb(self, goal):
        """Main action execution callback method."""
        num_joints=7
        coeffs = np.zeros((num_joints, 6))
        
        start_pose = self.tfBuffer.lookup_transform('base','panda_link8',rospy.Time())
        start_pos = [start_pose.transform.translation.x, start_pose.transform.translation.y, start_pose.transform.translation.z]
        start_ori = [start_pose.transfrom.rotation.x,start_pose.transfrom.rotation.y,start_pose.transfrom.rotation.z,start_pose.transfrom.rotation.w]

        start_quat_dict = dict()
        start_quat_dict['s'] = start_ori[3]
        start_quat_dict['v'] = np.asarray(start_ori[0:3])

        end_pos = goal.target_pose.position
        end_ori = goal.target_pose.orientation

        end_quat_dict = dict()
        end_quat_dict['s'] = end_ori[3]
        end_quat_dict['v'] = np.asarray(end_ori[0:3])

        motion_duration = goal.desired_travel_time
        motion_timestep = 0.1
        rate=rospy.Rate(10)

        # Generate the minimum jerk cartesian path trajectory
        coeffs_x = polynomial1(start_pos[0], 0.0, 0.0, end_pos[0], 0.0, 0.0, motion_duration)
        coeffs_y = polynomial1(start_pos[1], 0.0, 0.0, end_pos[1], 0.0, 0.0, motion_duration)
        coeffs_z = polynomial1(start_pos[2], 0.0, 0.0, end_pos[2], 0.0, 0.0, motion_duration)

        path_pos = np.zeros((int(motion_duration/motion_timestep + 1), 3))
        path_vel = np.zeros((int(motion_duration/motion_timestep + 1), 3))
        path_acc = np.zeros((int(motion_duration/motion_timestep + 1), 3))
        path_pos[:, 0], path_vel[:, 0], path_acc[:, 0] = polynomial2(coeffs_x, motion_timestep, motion_duration)
        path_pos[:, 1], path_vel[:, 1], path_acc[:, 1] = polynomial2(coeffs_y, motion_timestep, motion_duration)
        path_pos[:, 2], path_vel[:, 2], path_acc[:, 2] = polynomial2(coeffs_z, motion_timestep, motion_duration)
        
        path_quat = np.zeros((int(motion_duration/motion_timestep + 1), 4))
        for i in range(int(motion_duration/motion_timestep + 1)):
            i_t = i*motion_timestep
            path_quat_dict, _, _ = min_jerk_slerp(start_quat_dict, end_quat_dict, motion_duration, i_t)
            path_quat[i, 3] = path_quat_dict['s']
            path_quat[i, 0:3] = path_quat_dict['v'][0:3]


        # Publish info to the console for the user
        rospy.loginfo("[{0}]: Executing task space motion from current pose:\n{1} {2} to pose: \n{3} {4}"
                      " in {5} seconds.".format(self._action_name,
                                                           start_pos,start_ori,
                                                           end_pos,end_ori,
                                                           motion_duration))
        
        # Start executing the action
        for i in range(len(path_quat)):

            # Check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.logwarn('[{0}]: Preempted'.format(self._action_name))
                self._result.fully_succeeded = False
                self._as.set_preempted(self._result)
                return

            # Send command to the robot
            #self.move_robot_to_pose(path_pos[i, :], path_quat[i, :])
            print(path_pos[i, :])

            # Get the current pose as feedback
            self._feedback.current_pose = self.tfBuffer.lookup_transform('base','panda_link8',rospy.Time())
            #self._feedback.total_diff, pos_diff, rot_diff ...

            # Publish the feedback
            self._as.publish_feedback(self._feedback)
            rate.sleep()

        rospy.loginfo('[{0}]: Fully succeeded'.format(self._action_name))
        self._result.fully_succeeded = True
        self._as.set_succeeded(self._result)
        return


if __name__ == '__main__':

    try:
        rospy.init_node('cart_lin_action_server')
        server = CartLinActionInterface(rospy.get_name())

        rospy.loginfo("Action server \'{0}\' started.".format(rospy.get_name()))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
 
