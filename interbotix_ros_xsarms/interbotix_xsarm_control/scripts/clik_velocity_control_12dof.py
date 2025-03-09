#!/usr/bin/env python3
import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3Stamped, Pose
from std_msgs.msg import Float64MultiArray
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
from tf2_msgs.msg import TFMessage
import tf.transformations as tf_transformations
from interbotix_xsarm_control.msg import CartesianCommand  # Import the new message type
from scipy.spatial.transform import Rotation as R



class InverseKinematicsControl:
    def __init__(self):
        # Initialize the node
        rospy.init_node('ik_velocity_control', anonymous=True)        
        
        #Subscribers
        self.joint_state_sub = rospy.Subscriber("/mobile_wx250s/joint_states", JointState, self.joint_state_callback)
        # self.joint_state_sim_sub = rospy.Subscriber("/mobile_wx250s/joint_states", JointState, self.joint_state_sim_callback)
        # self.cartesian_vel_sub = rospy.Subscriber("/desired_cartesian_velocity", Twist, self.cartesian_vel_command_callback)
        self.cartesian_cmd_sub = rospy.Subscriber("/desired_cartesian_command", CartesianCommand, self.cartesian_command_callback)


        self.hexapod_joint_feedback = rospy.Subscriber('/joint_states', JointState ,self.hexapod_joint_feedback_callback)

        # Publishers
        self.ee_vel_pub = rospy.Publisher("/end_effector_velocity", Twist, queue_size=5)
        self.ee_pose_pub = rospy.Publisher("/ee_current_position", Pose, queue_size=5)
        self.pub = rospy.Publisher('/target_vel_hex', Float64MultiArray, queue_size=1)
        self.joint_vel_pub = rospy.Publisher("/mobile_wx250s/commands/joint_group", JointGroupCommand, queue_size=1)
        

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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
        self.qdot = np.zeros(12)
        self.clamped_joint_velocities = np.zeros(self.num_joints)
        self.joint_positions = np.zeros(self.num_joints)
        self.hexapod_min_limits = np.array([
            -0.05,   # x translation minimum (meters)
            -0.05,   # y translation minimum (meters)
            -0.025,  # z translation minimum (meters)
            -0.26,   # roll minimum (radians)
            -0.26,   # pitch minimum (radians)
            -0.52    # yaw minimum (radians)
        ])
        
        
        self.hexapod_max_limits = np.array([
            0.05,    # x translation maximum (meters)
            0.05,    # y translation maximum (meters)
            0.025,   # z translation maximum (meters)
            0.26,    # roll maximum (radians)
            0.26,    # pitch maximum (radians)
            0.52     # yaw maximum (radians)
        ])

        self.hexapod_min_limits = self.hexapod_min_limits * 1000
        self.hexapod_max_limits = self.hexapod_max_limits * 1000


        # Joint limits   |   Source: https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/wx250s.html 
        self.joint_position_limits_lower = np.deg2rad(np.array([-160, -108, -123, -180, -100, -180]))
        self.joint_position_limits_upper = np.deg2rad(np.array([160, 114, 92, 180, 123, 180]))
        


        

        # Set up solvers
        self.jacobian_solver = kdl.ChainJntToJacSolver(self.kdl_chain)
        self.joint_position_kdl = kdl.JntArray(self.num_joints)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)  # Forward Kinematics solver
        self.ee_current_pose = Pose()

        # Set the loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

        self.hexapod_joint_states_vector = [0,0,0,0,0,0]
        # self.ee_pose

        rospy.loginfo("Combined IK node initialized.")

    def weighted_pseudo_inverse(self, J_hexapod, J_arm, w_hexapod=1.0, w_arm=1.0):

        J_combined_weighted = np.hstack((w_hexapod*J_hexapod, w_arm*J_arm))

        J_weighted_pinv = np.linalg.pinv(J_combined_weighted)
        # J_weighted_pinv = np.linalg.pinv(J_combined_weighted.T @ J_combined_weighted + 0.01 * np.eye(J_combined_weighted.shape[1])) @ J_combined_weighted.T

        return J_weighted_pinv, J_combined_weighted


    def hexapod_joint_feedback_callback(self, msg):
        target_names = ['cart_x', 'cart_y', 'cart_z', 'ang_u', 'ang_v', 'ang_w']
        # Extract indices of target joints
        indices = [msg.name.index(name) for name in target_names if name in msg.name]

        # Extract positions of target joints using indices
        self.hexapod_joint_states_vector = [i for i in [msg.position[i] for i in indices]]
        # print(self.joint_states_vector)

    def skew_symmetric(self,v):
        """ Compute the skew-symmetric matrix of a vector v = (x, y, z) """
        x, y, z = v
        return np.array([
            [ 0, -z,  y],
            [ z,  0, -x],
            [-y,  x,  0]
        ])

    def get_hexapod_jacobian(self):
        """Compute the Jacobian matrix for the hexapod."""
        I3 = np.eye(3)  # 3x3 Identity matrix
        mount_position = np.array([0, 0, 17.0/1000])
        
        for i in range(self.num_joints):
            self.joint_position_kdl[i] = self.joint_positions[i]

        ee_frame = kdl.Frame()
        self.fk_solver.JntToCart(self.joint_position_kdl, ee_frame)

        ee_pose = Pose()
        ee_pose.position.x = ee_frame.p[0]
        ee_pose.position.y = ee_frame.p[1]
        ee_pose.position.z = ee_frame.p[2]
        
        # Convert the rotation to a quaternion
        rot = ee_frame.M.GetQuaternion()
        ee_pose.orientation.x = rot[0]
        ee_pose.orientation.y = rot[1]
        ee_pose.orientation.z = rot[2]
        ee_pose.orientation.w = rot[3]



        
        p_ee_H = np.array([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z ]) + mount_position
        skew_p_ee_H = self.skew_symmetric(p_ee_H)
        
        # Construct the full 6x6 Jacobian   
        J_hexapod = np.block([
            [I3, -skew_p_ee_H],
            [np.zeros((3,3)), I3]
        ])
        return J_hexapod
    
    
    def compute_joint_velocities(self, desired_cart_pos, cartesian_velocity, desired_cart_orientation):
        """
        Compute the joint velocities based on inverse kinematics, considering both position and velocity control.
        """
        # Compute actual end-effector position
        actual_cart_pos, actual_cart_orientation = self.get_end_effector_pose()

        # Compute position error - ensure it has the same dimensionality as cartesian_velocity
        pos_error = np.zeros(6)  # Initialize a 6D vector for position and orientation error
        pos_error[:3] = desired_cart_pos - actual_cart_pos  # First 3 elements are position error
        # print(pos_error)
        desired_euler = tf.transformations.euler_from_quaternion(desired_cart_orientation)
        actual_euler = tf.transformations.euler_from_quaternion(actual_cart_orientation)

        pos_error[3:] = np.array(desired_euler) - np.array(actual_euler)  # First 3 elements are position error
        print(pos_error[3:])
        # Kp = np.diag([5, 5, 5, 0.1, 0.1, 0.1])  # Position control gain (adjust as needed)
        Kp = 1
        # Compute joint velocities with position correction
        for i in range(self.num_joints):
            self.joint_position_kdl[i] = self.joint_positions[i]

        # Compute Jacobian
        jacobian = kdl.Jacobian(self.num_joints)
        self.jacobian_solver.JntToJac(self.joint_position_kdl, jacobian)

        jacobian_arm = np.zeros((6, self.num_joints))
        for i in range(6):
            for j in range(self.num_joints):
                jacobian_arm[i, j] = jacobian[i, j]

        jacobian_hexapod = self.get_hexapod_jacobian()
        # J_combined = np.hstack((jacobian_hexapod, jacobian_arm))
        Jpinv, J_combined = self.weighted_pseudo_inverse(jacobian_hexapod, jacobian_arm)

        # Primary task (task space control)
        # q_dot_task = Jpinv.dot(cartesian_velocity + Kp.dot(pos_error))
        q_dot_task = Jpinv.dot(cartesian_velocity + Kp*pos_error)


        
        # Null space projection matrix
        n = 12
        null_proj = np.eye(n) - Jpinv.dot(J_combined)

        # Current joint positions
        q = np.zeros(n)
        q[:6] = self.hexapod_joint_states_vector
        for i in range(self.num_joints):
            q[6 + i] = self.joint_position_kdl[i]

        # Joint limits
        q_min = np.zeros(n)
        q_max = np.zeros(n)

        # Define limits
        q_min[:6] = self.hexapod_min_limits  # hexapod min limits
        q_min[6:] = self.joint_position_limits_lower  # Arm lower limits
        q_max[:6] = self.hexapod_max_limits  # hexapod max limits
        q_max[6:] = self.joint_position_limits_upper  # Arm upper limits


        # Gradient of joint limit avoidance objective function
        gradient_w = self.calculate_gradient_phi(q, q_min, q_max)


        # K0 = 0.1
        K0 = 1e2*np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0,0,0,0,0,0])

        # Null space motion for joint limit avoidance
        q_dot_null = null_proj.dot(K0.dot(gradient_w))

        # print(q_dot_null)

        # Combined control law
        q_dot = q_dot_task #+ q_dot_null
        # q_dot = q_dot_task

        # Extract hexapod and arm velocities
        hexapod_vel = q_dot[:6]  # First 6 elements
        arm_vel = q_dot[6:]      # Remaining elements

        self.joint_velocities = arm_vel
        self.qdot = q_dot

        self.clamped_joint_velocities = np.clip(self.joint_velocities, -2.35, 2.35)
    
    def calculate_gradient_phi(self, q, q_min, q_max):
        """
        Calculate the gradient of the joint limit avoidance objective function.
        
        Args:
            q: Current joint positions
            q_min: Minimum joint limits
            q_max: Maximum joint limits
        
        Returns:
            gradient: Gradient of the objective function
        """
        n = len(q)
        q_mid = (q_max + q_min) / 2.0
        gradient = np.zeros(n)
        
        for i in range(n):
            # Calculate partial derivative of Φ with respect to q_i
            # Based on equation (12) from the reference materials
            denominator = (q_max[i] - q_min[i])**2
            if denominator > 0:  # Avoid division by zero
                normalized_distance = (q[i] - q_mid[i])**2 / denominator
                gradient[i] = -2 * (q[i] - q_mid[i]) / (n * denominator)
                # print(gradient[i])
        
        return gradient
    
    def get_end_effector_pose(self):
        """
        Get the current position and orientation of the end-effector.
        """
        ee_pos = np.array([
            self.ee_current_pose.position.x, 
            self.ee_current_pose.position.y, 
            self.ee_current_pose.position.z
        ])
        # print(ee_pos)

        ee_orientation = np.array([
            self.ee_current_pose.orientation.x, 
            self.ee_current_pose.orientation.y, 
            self.ee_current_pose.orientation.z, 
            self.ee_current_pose.orientation.w
        ])

        return ee_pos, ee_orientation


    # def compute_joint_velocities(self, cartesian_velocity):

    #     # rospy.loginfo(f"Desired Cartesian velocity: {cartesian_velocity}")

    #     for i in range(self.num_joints):
    #         self.joint_position_kdl[i] = self.joint_positions[i]
        
    #     # rospy.loginfo(f"Joint states: {self.joint_position_kdl}")

    #     # Compute the Jacobian using KDL
    #     jacobian = kdl.Jacobian(self.num_joints)
    #     self.jacobian_solver.JntToJac(self.joint_position_kdl, jacobian)

    #     # Log the Jacobian
    #     jacobian_arm = np.zeros((6, self.num_joints))
    #     for i in range(6):
    #         for j in range(self.num_joints):
    #             jacobian_arm[i, j] = jacobian[i, j]

    #     jacobian_hexapod = self.get_hexapod_jacobian()
    #     J_combined = np.hstack((jacobian_hexapod, jacobian_arm))
    #     # J_combined = jacobian_arm
        
    #     Jpinv = self.weighted_pseudo_inverse(jacobian_hexapod, jacobian_arm)

    #     # Solve for velocities using pseudo-inverse
    #     # q_dot = np.linalg.pinv(J_combined).dot(cartesian_velocity)
    #     q_dot = Jpinv.dot(cartesian_velocity)
        
    #     # Kp = 0.1
    #     # xd = 
    #     # x = 
    #     # q_dot = Jpinv.dot(cartesian_velocity + Kp*(xd-x))




    #     # Extract hexapod and arm velocities
    #     hexapod_vel = q_dot[:6]  # First 6 elements
    #     arm_vel = q_dot[6:]      # Remaining elements

    #     self.joint_velocities = arm_vel
    #     self.qdot = q_dot
        
    #     # rospy.loginfo(f"---------------------------------------------------------------------------- {0}")
    #     # rospy.loginfo(f"jacobian: {jacobian_hexapod}")
    #     # rospy.loginfo(f"arm velocities: {arm_vel}")
    #     # rospy.loginfo(f"hexapod velocities: {hexapod_vel}")
    #     # rospy.loginfo(f"---------------------------------------------------------------------------- {0}")
    #     # rospy.loginfo(f"joint velocities: {self.joint_velocities}")



    #     self.clamped_joint_velocities =  np.clip(self.joint_velocities, -2.35, 2.35) # Check these limits - these are ideal values (no load condition)
    
    def compute_rotation_matrix(self, ang_u_mrad, ang_v_mrad, ang_w_mrad):
        """
        Compute the rotation matrix from hexapod platform to world frame
        given angular displacements in milliradians.

        Parameters:
            ang_u_mrad (float): Rotation about x-axis (roll) in milliradians.
            ang_v_mrad (float): Rotation about y-axis (pitch) in milliradians.
            ang_w_mrad (float): Rotation about z-axis (yaw) in milliradians.

        Returns:
            np.ndarray: 3x3 rotation matrix.
        """
        # Convert milliradians to radians
        ang_u = ang_u_mrad / 1000.0  # Roll (X-axis)
        ang_v = ang_v_mrad / 1000.0  # Pitch (Y-axis)
        ang_w = ang_w_mrad / 1000.0  # Yaw (Z-axis)

        # Rotation matrix around X-axis (Roll)
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(ang_u), -np.sin(ang_u)],
                        [0, np.sin(ang_u), np.cos(ang_u)]])

        # Rotation matrix around Y-axis (Pitch)
        R_y = np.array([[np.cos(ang_v), 0, np.sin(ang_v)],
                        [0, 1, 0],
                        [-np.sin(ang_v), 0, np.cos(ang_v)]])

        # Rotation matrix around Z-axis (Yaw)
        R_z = np.array([[np.cos(ang_w), -np.sin(ang_w), 0],
                        [np.sin(ang_w), np.cos(ang_w), 0],
                        [0, 0, 1]])

        # Final rotation matrix (Extrinsic rotation: Rz * Ry * Rx)
        R_hexapod_to_world = R_z @ R_y @ R_x

        return R_hexapod_to_world

    def compute_end_effector_pose(self):
        # Step 1: Forward Kinematics to get EE pose in arm frame
        for i in range(self.num_joints):
            self.joint_position_kdl[i] = self.joint_positions[i]

        ee_frame = kdl.Frame()
        self.fk_solver.JntToCart(self.joint_position_kdl, ee_frame)

        # Step 2: Set the EE position and orientation
        ee_pose = Pose()
        ee_pose.position.x = ee_frame.p[0]
        ee_pose.position.y = ee_frame.p[1]
        ee_pose.position.z = ee_frame.p[2]

        # Convert the rotation to a quaternion
        rot = ee_frame.M.GetQuaternion()
        ee_pose.orientation.x = rot[0]
        ee_pose.orientation.y = rot[1]
        ee_pose.orientation.z = rot[2]
        ee_pose.orientation.w = rot[3]

        # rospy.loginfo(f"FK Computed EE Pose (Before Transform): x={ee_pose.position.x}, y={ee_pose.position.y}, z={ee_pose.position.z}")

        try:
            # Wait for the transform to be available
            transform = self.tf_buffer.lookup_transform('base_link', 'mobile_wx250s/base_link', rospy.Time(0), rospy.Duration(1))
            transform2 = self.tf_buffer.lookup_transform('base_link', 'mobile_wx250s/ee_arm_link', rospy.Time(0), rospy.Duration(1))

            # Extract translation
            trans = transform.transform.translation
            translation = [trans.x, trans.y, trans.z]

            # Extract rotation (quaternion)
            rot = transform.transform.rotation
            rotation = [rot.x, rot.y, rot.z, rot.w]
            
            
            # rospy.loginfo(f"TF Lookup Translation: x={trans.x}, y={trans.y}, z={trans.z}")
            # rospy.loginfo(f"TF Lookup Rotation (quaternion): {rotation}")

            # Convert to homogeneous transformation matrix
            trans_matrix = tf_transformations.quaternion_matrix(rotation)
            trans_matrix[:3, 3] = [trans.x, trans.y, trans.z]  # Set translation

            # Convert EE position to homogeneous coordinates
            ee_position = np.array([ee_pose.position.x, ee_pose.position.y, ee_pose.position.z, 1.0])

            # Apply the transform correctly
            transformed_position = trans_matrix @ ee_position

            # Extract transformed position
            transformed_pose = Pose()
            transformed_pose.position.x = transformed_position[0]
            transformed_pose.position.y = transformed_position[1]
            transformed_pose.position.z = transformed_position[2]

            # Apply rotation to orientation
            transformed_orientation = tf_transformations.quaternion_multiply(rotation, 
                                                [ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z, ee_pose.orientation.w])

            # Extract transformed orientation
            transformed_pose.orientation.x = transformed_orientation[0]
            transformed_pose.orientation.y = transformed_orientation[1]
            transformed_pose.orientation.z = transformed_orientation[2]
            transformed_pose.orientation.w = transformed_orientation[3]

            # Publish the transformed pose
            # self.ee_pose_pub.publish(transformed_pose)
            
            # Create a Pose message
            pose2 = Pose()

            # Set position
            pose2.position.x = transform2.transform.translation.x
            pose2.position.y = transform2.transform.translation.y
            pose2.position.z = transform2.transform.translation.z

            # Set orientation (quaternion)
            pose2.orientation.x = transform2.transform.rotation.x
            pose2.orientation.y = transform2.transform.rotation.y
            pose2.orientation.z = transform2.transform.rotation.z
            pose2.orientation.w = transform2.transform.rotation.w


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Could not transform ee_pose from {} to {}".format('mobile_wx250s/base_link','base_link'))
            # return None

        self.ee_current_pose = pose2
        self.ee_pose_pub.publish(pose2)


    def cartesian_command_callback(self, msg):
        """
        Callback function for the desired Cartesian position and velocity command.
        """
        desired_cart_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
        desired_cart_orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        desired_cartesian_velocity = np.array([
            msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z,
            msg.velocity.angular.x, msg.velocity.angular.y, msg.velocity.angular.z
        ])
        
        # Compute joint velocities using inverse kinematics
        self.compute_joint_velocities(desired_cart_pos, desired_cartesian_velocity, desired_cart_orientation)



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
        
        # Compute and publish the current end-effector pose
        self.compute_end_effector_pose()
        
        
    def apply_joint_limits(self, joint_velocities):
        '''
        apply_joint_limits(self, joint_velocities)
                This function checks if joint position limits are reached, and if so, it sets the corresponding joint velocity to zero.
        '''
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
        
        hexapod_vel = self.qdot[:6]  # First 6 elements
        msg = Float64MultiArray()
        msg.data = [hexapod_vel[0], hexapod_vel[1], hexapod_vel[2], hexapod_vel[3], hexapod_vel[4], hexapod_vel[5], 10.0]
        print(msg.data)
        
        
        # Publish the message
        
        self.joint_vel_pub.publish(joint_cmd_msg)

        self.pub.publish(msg)
        
        # rospy.loginfo(f"---------------------------------------------------------------------------- {0}")
        # # rospy.loginfo(f"jacobian: {jacobian_hexapod}")
        # rospy.loginfo(f"arm velocities: {self.joint_velocities}")
        # rospy.loginfo(f"hexapod velocities: {hexapod_vel}")
        # rospy.loginfo(f"---------------------------------------------------------------------------- {0}")



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
