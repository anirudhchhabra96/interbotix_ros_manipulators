#!/usr/bin/env python3
import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3Stamped
import geometry_msgs
from interbotix_xs_msgs.msg import JointGroupCommand  # Import the correct message type
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_vector3
import geometry_msgs.msg
import tf
import os

class InverseKinematicsControl:
    def __init__(self):
        # Initialize the node
        rospy.init_node('ik_velocity_control', anonymous=True)

        # Subscribe to the cartesian velocity command topic
        self.cartesian_vel_sub = rospy.Subscriber("/desired_cartesian_velocity", Twist, self.cartesian_vel_command_callback)

        # Publish to the joint velocity command topic
        self.joint_vel_pub = rospy.Publisher("/mobile_wx250s/commands/joint_group", JointGroupCommand, queue_size=10)

        # Subscribe to joint state topic to get the current joint positions
        self.joint_state_sub = rospy.Subscriber("/mobile_wx250s/joint_states", JointState, self.joint_state_callback)

        self.joint_state_sim_sub = rospy.Subscriber("/joint_states_sim", JointState, self.joint_state_sim_callback)

        # Publish the end-effector velocity
        self.ee_vel_pub = rospy.Publisher("/end_effector_velocity", Twist, queue_size=10)

        # Retrieve the robot description from the ROS parameter server
        urdf_param = "/mobile_wx250s/robot_description"
        urdf_string = rospy.get_param(urdf_param)

        # Build the KDL tree from the URDF string
        (ok, self.kdl_tree) = treeFromParam(urdf_param)
        if not ok:
            rospy.logerr("Failed to extract KDL tree from the robot description")

        self.kdl_chain = self.kdl_tree.getChain("mobile_wx250s/base_link", "mobile_wx250s/ee_arm_link")

        # Number of joints
        self.num_joints = self.kdl_chain.getNrOfJoints()
        self.joint_velocities = np.zeros(self.num_joints)
        self.clamped_joint_velocities = np.zeros(self.num_joints)
        self.joint_positions = np.zeros(self.num_joints)

        # Joint limits   |   Source: https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx250s.html 
        self.joint_position_limits_lower = np.deg2rad(np.array([-180, -108, -123, -180, -100, -180]))
        self.joint_position_limits_upper = np.deg2rad(np.array([180, 114, 92, 180, 123, 180]))

        # Set up solvers
        self.jacobian_solver = kdl.ChainJntToJacSolver(self.kdl_chain)
        self.joint_position_kdl = kdl.JntArray(self.num_joints)

        # Set the loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def compute_joint_velocities(self, cartesian_velocity):

        # rospy.loginfo(f"Desired Cartesian velocity: {cartesian_velocity}")

        for i in range(self.num_joints):
            self.joint_position_kdl[i] = self.joint_positions[i]
        
        # rospy.loginfo(f"Joint states: {self.joint_position_kdl}")

        # Compute the Jacobian using KDL
        jacobian = kdl.Jacobian(self.num_joints)
        self.jacobian_solver.JntToJac(self.joint_position_kdl, jacobian)

        # Log the Jacobian
        jacobian_array = np.zeros((6, self.num_joints))
        for i in range(6):
            for j in range(self.num_joints):
                jacobian_array[i, j] = jacobian[i, j]
        
        # Compute joint velocities using Jacobian-based IK methods
        try:    
            ##--------------------------------------------------------------------------------------
            ##              Using Jacobian Pseudoinverse
            self.joint_velocities = np.linalg.pinv(jacobian_array).dot(cartesian_velocity)
            ##--------------------------------------------------------------------------------------
            
            ##--------------------------------------------------------------------------------------
            ##              Using Damped Jacobians
            # damping_factor = 0.01
            # jacobian_damped = jacobian_array.T @ np.linalg.inv(jacobian_array @ jacobian_array.T + damping_factor * np.identity(6))
            # self.joint_velocities = np.dot(jacobian_damped, cartesian_velocity)
            ##--------------------------------------------------------------------------------------
        except np.linalg.LinAlgError:
            rospy.logwarn("Jacobian is singular, unable to compute joint velocities.")
            self.joint_velocities = np.zeros(self.num_joints)
        
        self.clamped_joint_velocities =  np.clip(self.joint_velocities, -2.35, 2.35) # Check these limits - these are ideal values (no load condition)

    def cartesian_vel_command_callback(self, msg):
        msg1 = msg
        desired_cartesian_velocity = [msg1.linear.x, msg1.linear.y, msg1.linear.z, msg1.angular.x, msg1.angular.y, msg1.angular.z]
        # rospy.loginfo(f"Received cartesian velocity command: {desired_cartesian_velocity}")
        self.compute_joint_velocities(desired_cartesian_velocity)

    def joint_state_sim_callback(self, msg):
        self.compute_end_effector_velocity(msg.velocity)

    def joint_state_callback(self, msg):
        # Initialize the joint positions array if not already initialized
        
        if len(self.joint_positions) != len(msg.position):
            self.joint_positions = np.zeros(len(msg.position))

        # Map the joint names to indices in the message
        joint_map = {name: i for i, name in enumerate(msg.name)}

        # Ensure joint names match the expected KDL chain joints and update positions
        for i, joint_name in enumerate(['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']):
            if joint_name in joint_map:
                self.joint_positions[i] = msg.position[joint_map[joint_name]]

        # Log the joint positions
        # rospy.loginfo(f"Updated joint positions: {self.joint_positions}")
        
    def compute_end_effector_velocity(self, joint_velocities):
        # Update joint positions in KDL
        for i in range(self.num_joints):
            self.joint_position_kdl[i] = self.joint_positions[i]

        # Compute the Jacobian
        jacobian = kdl.Jacobian(self.num_joints)
        self.jacobian_solver.JntToJac(self.joint_position_kdl, jacobian)

        # Convert KDL Jacobian to numpy array
        jacobian_array = np.zeros((6, self.num_joints))
        for i in range(6):
            for j in range(self.num_joints):
                jacobian_array[i, j] = jacobian[i, j]

        # Compute end-effector velocity
        ee_velocity = jacobian_array.dot(joint_velocities)
        
        # Create and publish the Twist message for end-effector velocity
        ee_vel_msg = Twist()
        ee_vel_msg.linear.x = ee_velocity[0]
        ee_vel_msg.linear.y = ee_velocity[1]
        ee_vel_msg.linear.z = ee_velocity[2]
        ee_vel_msg.angular.x = ee_velocity[3]
        ee_vel_msg.angular.y = ee_velocity[4]
        ee_vel_msg.angular.z = ee_velocity[5]
        
        self.ee_vel_pub.publish(ee_vel_msg)
        
    '''
    apply_joint_limits(self, joint_velocities)
            This function checks if joint position limits are reached, and if so, it sets the corresponding joint velocity to zero.
    '''
    
    def apply_joint_limits(self, joint_velocities):
        # Ensure joint positions stay within limits
        for i in range(self.num_joints):
            
            # rospy.loginfo(f"Joint {i}: position = {self.joint_positions[i]}, velocity = {joint_velocities[i]}, limits = ({self.joint_position_limits_lower[i]}, {self.joint_position_limits_upper[i]})")
            if self.joint_positions[i] >= self.joint_position_limits_upper[i] and joint_velocities[i] > 0:
                joint_velocities[i] = 0  # Stop positive motion at upper limit
            elif self.joint_positions[i] <= self.joint_position_limits_lower[i] and joint_velocities[i] < 0:
                joint_velocities[i] = 0  # Stop negative motion at lower limit
        return joint_velocities
    
    '''
    publish_joint_velocities(self)
            This function publishes the calculated joint velocity commands to the /mobile_wx250s/commands/joint_group topic.
    '''

    def publish_joint_velocities(self):
        
        # rospy.loginfo("entered publish function")
        self.joint_velocities = self.apply_joint_limits(self.clamped_joint_velocities)

        # Create and populate the JointGroupCommand message
        joint_cmd_msg = JointGroupCommand()
        joint_cmd_msg.name = "arm"  # Name of the joint group (replace if necessary)
        joint_cmd_msg.cmd = self.joint_velocities.tolist()  # List of joint velocities
                
        # Publish the message
        self.joint_vel_pub.publish(joint_cmd_msg)

    def run(self):
        while not rospy.is_shutdown():
            # self.compute_joint_velocities()  # Compute and publish joint velocities
            self.publish_joint_velocities()
            # os.system('clear')
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ik_control = InverseKinematicsControl()
        ik_control.run()
    except rospy.ROSInterruptException:
        pass
