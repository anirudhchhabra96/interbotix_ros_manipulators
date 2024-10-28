#!/usr/bin/env python
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
        self.joint_positions = np.zeros(self.num_joints)

        # Joint limits (confirm and update these)
        self.joint_position_limits_lower = np.array([-3.0, -3.0, -3.0, -3.0, -3.0, -3.0])
        self.joint_position_limits_upper = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0])

        # Set up solvers
        self.jacobian_solver = kdl.ChainJntToJacSolver(self.kdl_chain)
        self.joint_position_kdl = kdl.JntArray(self.num_joints)

        # Set the loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def compute_joint_velocities(self, cartesian_velocity):

        rospy.loginfo(f"Desired Cartesian velocity: {cartesian_velocity}")

        for i in range(self.num_joints):
            self.joint_position_kdl[i] = self.joint_positions[i]

        # Compute the Jacobian using KDL
        jacobian = kdl.Jacobian(self.num_joints)
        self.jacobian_solver.JntToJac(self.joint_position_kdl, jacobian)

        # Log the Jacobian
        jacobian_array = np.zeros((6, self.num_joints))
        for i in range(6):
            for j in range(self.num_joints):
                jacobian_array[i, j] = jacobian[i, j]
        # rospy.loginfo(f"Jacobian: \n{jacobian_array}")

        try:
            # Compute joint velocities using the inverse Jacobian
            self.joint_velocities = np.linalg.pinv(jacobian_array).dot(cartesian_velocity)
            # rospy.loginfo(f"Computed joint velocities: {self.joint_velocities}")
        except np.linalg.LinAlgError:
            rospy.logwarn("Jacobian is singular, unable to compute joint velocities.")
            self.joint_velocities = np.zeros(self.num_joints)
        
        self.clamped_joint_velocities =  np.clip(self.joint_velocities, -0.2, 0.2)
        # self.clamped_joint_velocities =  self.joint_velocities

        self.joint_velocities = self.apply_joint_limits(self.clamped_joint_velocities)
    
    def do_transform_twist(self, twist, transform):
        # Convert rotation (quaternion) from the transform into a numpy array
        
        # Transform linear velocity
        linear_velocity = Vector3Stamped()
        linear_velocity.vector = twist.linear
        linear_velocity.header.frame_id = transform.header.frame_id
    
        # Use tf2 to transform the linear velocity
        transformed_linear = tf2_geometry_msgs.do_transform_vector3(linear_velocity, transform)
        
        # Transform angular velocity
        angular_velocity = Vector3Stamped()
        angular_velocity.vector = twist.angular
        angular_velocity.header.frame_id = transform.header.frame_id
        
        # Use tf2 to transform the angular velocity
        transformed_angular = tf2_geometry_msgs.do_transform_vector3(angular_velocity, transform)
            
        # Create new Twist message with the transformed velocities
        transformed_twist = Twist()
        transformed_twist.linear = transformed_linear.vector
        transformed_twist.angular = transformed_angular.vector
    
        return transformed_twist
    
    def cartesian_vel_command_callback(self, msg):
        
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            transform_base_ee = tf_buffer.lookup_transform("mobile_wx250s/base_link","mobile_wx250s/ee_arm_link", rospy.Time(0), rospy.Duration(1.0))
            transformed_cart_vel_in_base_frame = self.do_transform_twist(msg, transform_base_ee)
            rospy.loginfo(f"transformed velocity: {transformed_cart_vel_in_base_frame}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Could not do transform: {e}")
        
        msg1 = transformed_cart_vel_in_base_frame
        desired_cartesian_velocity = [msg1.linear.x, msg1.linear.y, msg1.linear.z, msg1.angular.x, msg1.angular.y, msg1.angular.z]
        rospy.loginfo(f"Received cartesian velocity command: {desired_cartesian_velocity}")
        self.compute_joint_velocities(desired_cartesian_velocity)

    
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




    def apply_joint_limits(self, joint_velocities):
        # Ensure joint positions stay within limits
        for i in range(self.num_joints):
            
            # rospy.loginfo(f"Joint {i}: position = {self.joint_positions[i]}, velocity = {joint_velocities[i]}, limits = ({self.joint_position_limits_lower[i]}, {self.joint_position_limits_upper[i]})")
            
            if self.joint_positions[i] >= self.joint_position_limits_upper[i] and joint_velocities[i] > 0:
                joint_velocities[i] = 0  # Stop positive motion at upper limit
            elif self.joint_positions[i] <= self.joint_position_limits_lower[i] and joint_velocities[i] < 0:
                joint_velocities[i] = 0  # Stop negative motion at lower limit
        return joint_velocities

    def publish_joint_velocities(self):
        
        # rospy.loginfo("entered publish function")

        # Create and populate the JointGroupCommand message
        joint_cmd_msg = JointGroupCommand()
        joint_cmd_msg.name = "arm"  # Name of the joint group (replace if necessary)
        
        joint_cmd_msg.cmd = self.joint_velocities.tolist()  # List of joint velocities
        
        # Logging
        # rospy.loginfo(f"Publishing joint velocities: {joint_cmd_msg.cmd} for group 'arm'")
        # rospy.loginfo(f"Sending joint velocity command: {command.cmd}")
        
        # Publish the message
        self.joint_vel_pub.publish(joint_cmd_msg)
        



    def run(self):
        while not rospy.is_shutdown():
            # self.compute_joint_velocities()  # Compute and publish joint velocities
            self.publish_joint_velocities()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ik_control = InverseKinematicsControl()
        ik_control.run()
    except rospy.ROSInterruptException:
        pass
