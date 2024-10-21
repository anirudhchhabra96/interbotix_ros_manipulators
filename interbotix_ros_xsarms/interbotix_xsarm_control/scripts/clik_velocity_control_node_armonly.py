#!/usr/bin/env python3
import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointGroupCommand  # Import the correct message type
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam

class InverseKinematicsControl:
    def __init__(self):
        # Initialize the node
        rospy.init_node('ik_velocity_control', anonymous=True)

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
        # rospy.loginfo(f"KDL Tree successfully built: {self.kdl_tree}")
        # rospy.loginfo(f"KDL Chain from 'base_link' to 'ee_arm_link' with {self.kdl_chain.getNrOfJoints()} joints.")


        # Number of joints
        self.num_joints = self.kdl_chain.getNrOfJoints()
        self.joint_velocities = np.zeros(self.num_joints)
        self.joint_positions = np.zeros(self.num_joints)

        # Joint limits (confirm and update these)
        self.joint_position_limits_lower = np.array([-3.0, -3.0, -3.0, -3.0, -3.0, -3.0])
        self.joint_position_limits_upper = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0])

        # Define desired Cartesian velocity
        self.desired_linear_velocity = np.array([0.1, 0.0, 0.1])  # 0.1 m/s in x-direction
        self.desired_angular_velocity = np.array([0.0, 0.0, 0.0])  # 0.1 rad/s around z-axis

        # Set up solvers
        self.jacobian_solver = kdl.ChainJntToJacSolver(self.kdl_chain)
        self.joint_position_kdl = kdl.JntArray(self.num_joints)

        # Set the loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def compute_joint_velocities(self):
        cartesian_velocity = np.hstack((self.desired_linear_velocity, self.desired_angular_velocity))
        # rospy.loginfo(f"Desired Cartesian velocity: {cartesian_velocity}")

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
        rospy.loginfo("entered publish function")
        # # Convert numpy array to Float64MultiArray message
        # joint_vel_msg = Float64MultiArray()
        # joint_vel_msg.data = self.joint_velocities.tolist()

        # # Publish joint velocities
        # self.joint_vel_pub.publish(joint_vel_msg)

        # Create and populate the JointGroupCommand message
        joint_cmd_msg = JointGroupCommand()
        joint_cmd_msg.name = "arm"  # Name of the joint group (replace if necessary)
        
        joint_cmd_msg.cmd = self.joint_velocities.tolist()  # List of joint velocities
        
        # Log for debugging
        rospy.loginfo(f"Publishing joint velocities: {joint_cmd_msg.cmd} for group 'arm'")
        # rospy.loginfo(f"Sending joint velocity command: {command.cmd}")
        # Publish the message
        self.joint_vel_pub.publish(joint_cmd_msg)
        # rospy.loginfo(f"Publishing joint velocities: {joint_cmd_msg}")
        # self.joint_vel_pub.publish(joint_cmd_msg)
        rospy.loginfo("Joint velocities published")




    def run(self):
        while not rospy.is_shutdown():
            self.compute_joint_velocities()  # Compute and publish joint velocities
            self.publish_joint_velocities()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ik_control = InverseKinematicsControl()
        ik_control.run()
    except rospy.ROSInterruptException:
        pass
