#! /usr/bin/env python
#
# Action server that sends commands to cartesian impedance controller from ijs_controllers.
#
# Authors: Mihael Simonic
# Edited: Matej Štefanič
#

# Calculations
from minimum_jerk import polynomial1, polynomial2
from minimum_jerk_slerp import min_jerk_slerp

# Numpy
import numpy as np

# KDL
from kdl_tree_urdf_model import *
import PyKDL

# ROS stuff
import rospy
import actionlib
import tf2_ros

# ROS messages
from sensor_msgs.msg import JointState
from franka_core_msgs.msg import JointCommand, RobotState

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
    _jointposition = []

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
        
        # KDL
        base_link = 'base'
        tip_link = 'panda_link8'
        kdl_tree = kdl_from_param_server('/joint_states')
        arm_chain = kdl_tree.getChain(base_link, tip_link)
        num_jnts = 7
        self.jac_kdl = PyKDL.Jacobian(num_jnts)
        self.jac_kdl_solver = PyKDL.ChainJntToJacSolver(arm_chain)

        # Initialize publisher
        self._command_pub = rospy.Publisher('/panda_simulator/motion_controller/arm/joint_commands',JointCommand, queue_size = 1, tcp_nodelay = True)
        
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
    def move_robot_to_point(self, pos_desired): #, vel_desired):
        
        # Convert to numpy
        q = np.asarray(pos_desired)  # make inverse kinematic here for q - joints
        #qdot = np.asarray(vel_desired)

        #Construct message
        cmd_msg = robot_module_msgs.msg.JointCommand()
        cmd_msg.target_pose = q
        cmd_msg.desired_travel_time = 2
        cmd_msg.blend_radius = 0
        
        #Publish
        self._command_pub.publish(cmd_msg)
        
        rospy.logdebug("Next point sent")

    # ------------------------------------------
    # Main action server execution callback
    # ------------------------------------------
    def execute_cb(self, goal):
        """Main action execution callback method."""
        num_joints=7
        coeffs = np.zeros((num_joints, 6)) # 7x6
        
        start_pose = self.tfBuffer.lookup_transform('base','panda_link8',rospy.Time()) # pos of panda_link8 based on base at current time
        start_pos = [start_pose.transform.translation.x, start_pose.transform.translation.y, start_pose.transform.translation.z] # read from start_pose
        start_ori = [start_pose.transfrom.rotation.x,start_pose.transfrom.rotation.y,start_pose.transfrom.rotation.z,start_pose.transfrom.rotation.w] # read from start_pose

        start_quat_dict = dict()
        start_quat_dict['s'] = start_ori[3]
        start_quat_dict['v'] = np.asarray(start_ori[0:3])

        end_pos = goal.target_pose.position
        end_ori = goal.target_pose.orientation

        end_quat_dict = dict() # compatibilty for function
        end_quat_dict['s'] = end_ori[3]
        end_quat_dict['v'] = np.asarray(end_ori[0:3])

        motion_duration = goal.desired_travel_time
        motion_timestep = 0.02
        rate=rospy.Rate(50)

        # Generate the minimum jerk cartesian path trajectory
        coeffs_x = polynomial1(start_pos[0], 0.0, 0.0, end_pos[0], 0.0, 0.0, motion_duration)
        coeffs_y = polynomial1(start_pos[1], 0.0, 0.0, end_pos[1], 0.0, 0.0, motion_duration)
        coeffs_z = polynomial1(start_pos[2], 0.0, 0.0, end_pos[2], 0.0, 0.0, motion_duration)

        path_pos = np.zeros((int(motion_duration/motion_timestep + 1), 3))
        path_vel = np.zeros((int(motion_duration/motion_timestep + 1), 3))
        path_acc = np.zeros((int(motion_duration/motion_timestep + 1), 3))
        path_pos[:, 0], path_vel[:, 0], path_acc[:, 0] = polynomial2(coeffs_x, motion_timestep, motion_duration) # creates trajectory
        path_pos[:, 1], path_vel[:, 1], path_acc[:, 1] = polynomial2(coeffs_y, motion_timestep, motion_duration)
        path_pos[:, 2], path_vel[:, 2], path_acc[:, 2] = polynomial2(coeffs_z, motion_timestep, motion_duration)
        
        path_quat = np.zeros((int(motion_duration/motion_timestep + 1), 4)) # prepare matrix
        for i in range(int(motion_duration/motion_timestep + 1)):
            i_t = i*motion_timestep
            path_quat_dict, _, _ = min_jerk_slerp(start_quat_dict, end_quat_dict, motion_duration, i_t) #interpolation
            path_quat[i, 3] = path_quat_dict['s'] # s - scalar
            path_quat[i, 0:3] = path_quat_dict['v'][0:3] # v - vector


        # Publish info to the console for the user
        rospy.loginfo("[{0}]: Executing task space motion from current pose:\n{1} {2} to pose: \n{3} {4}"
                      " in {5} seconds.".format(self._action_name,
                                                           start_pos,start_ori,
                                                           end_pos,end_ori,
                                                           motion_duration))
        
        # get initial joint configuration
        init_state = rospy.wait_for_message('/joint_states', JointState, timeout=1)
        last_joint_conf_kdl = PyKDL.JntArray(self.num_jnts)
        for i in range(self.num_jnts):
            last_joint_conf_kdl[i] = init_state.position[i]

        traj= JointCommand()
        joint_names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        traj.joint_names = joint_names 

        time = 1

        # Start executing the action
        for i in range(len(path_quat)):

            self.jac_kdl_solver.JntToJac(last_joint_conf_kdl, self.jac_kdl)
            jac = kdl_to_mat(self.jac_kdl)
            jac_pseudo_inverse = np.linalg.pinv(jac)

            change = np.appned(end_pos, end_ori)

            last_joint_conf = [q for q in last_joint_conf_kdl];
            desired_joint_conf = last_joint_conf + np.matmul(jac_pseudo_inverse, change)

            trjp = JointCommand()
            trjp.position = desired_joint_conf.A1.tolist()
            trjp.time_from_start = rospy.Duration(time)
            traj.points.append(trjp)

            time += 0.1
            
            # Check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.logwarn('[{0}]: Preempted'.format(self._action_name))
                self._result.fully_succeeded = False
                self._as.set_preempted(self._result)
                return

            # Send command to the robot
            #self.move_robot_to_pose(path_pos[i, :], path_quat[i, :])
            print(path_pos[i, :])
            print(traj.points[i, :])

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
 
