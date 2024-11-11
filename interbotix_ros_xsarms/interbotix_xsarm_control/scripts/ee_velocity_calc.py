#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import numpy as np

# Define joint names and create a global variable to store joint velocities
joint_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
joint_velocities = np.zeros(len(joint_names))

# Placeholder function to compute the Jacobian matrix for the robot
def compute_jacobian(joint_positions):
    """
    Compute the Jacobian matrix for the robot at the given joint positions.
    Replace this function with your actual Jacobian computation method.
    """
    # Example: Return a dummy Jacobian matrix (6x6 identity matrix for simplicity)
    return np.identity(6)

def joint_state_callback(msg):
    global joint_velocities

    # Map joint names to indices in the message
    joint_map = {name: i for i, name in enumerate(msg.name)}

    # Update joint velocities based on the message data
    for i, joint_name in enumerate(joint_names):
        if joint_name in joint_map:
            joint_index = joint_map[joint_name]
            joint_velocities[i] = msg.velocity[joint_index]

def publish_end_effector_velocity():
    # Initialize node and publishers
    rospy.init_node('end_effector_velocity_publisher')
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    ee_velocity_pub = rospy.Publisher('/end_effector_velocity', TwistStamped, queue_size=10)

    rate = rospy.Rate(10)  # Publish rate in Hz

    while not rospy.is_shutdown():
        # Compute the Jacobian matrix based on the current joint positions
        joint_positions = np.zeros(len(joint_names))  # Placeholder for joint positions if needed
        jacobian = compute_jacobian(joint_positions)

        # Calculate the end-effector velocity
        end_effector_velocity = np.dot(jacobian, joint_velocities)

        # Create and populate the TwistStamped message
        ee_velocity_msg = TwistStamped()
        ee_velocity_msg.header.stamp = rospy.Time.now()
        ee_velocity_msg.twist.linear.x = end_effector_velocity[0]
        ee_velocity_msg.twist.linear.y = end_effector_velocity[1]
        ee_velocity_msg.twist.linear.z = end_effector_velocity[2]
        ee_velocity_msg.twist.angular.x = end_effector_velocity[3]
        ee_velocity_msg.twist.angular.y = end_effector_velocity[4]
        ee_velocity_msg.twist.angular.z = end_effector_velocity[5]

        # Publish the end-effector velocity
        ee_velocity_pub.publish(ee_velocity_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_end_effector_velocity()
    except rospy.ROSInterruptException:
        pass
