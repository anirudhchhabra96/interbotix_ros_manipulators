#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
import math
import tf
import tf.transformations

class VelocityProfileNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('velocity_profile_node', anonymous=True)

        # Parameters
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.2)  # Max linear speed in m/s
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.2)  # Max angular speed in rad/s

        # Current and target positions
        self.current_position = Pose()
        self.target_position = Pose()
        
        # Publishers and subscribers
        self.vel_pub = rospy.Publisher('/desired_cartesian_velocity', Twist, queue_size=10)
        

        rospy.Subscriber('/ee_current_position', Pose, self.current_position_callback)
        rospy.Subscriber('/ee_target_position', Pose, self.target_position_callback)

    def current_position_callback(self, msg):
        self.current_position = msg
        self.calculate_and_publish_velocity()

    def target_position_callback(self, msg):
        self.target_position = msg
        self.calculate_and_publish_velocity()

    def calculate_and_publish_velocity(self):
        # Calculate the differences in position
        dx = self.target_position.position.x - self.current_position.position.x
        dy = self.target_position.position.y - self.current_position.position.y
        dz = self.target_position.position.z - self.current_position.position.z

        ##--------------------------------------------------------------------------------------
        ##--------------------------------------------------------------------------------------

        # Calculate distances and normalize for linear velocity
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance > 0:
            linear_scale = min(self.max_linear_speed / distance, 1.0)
        else:
            linear_scale = 0

        # Calculate linear velocities
        vx = linear_scale * dx
        vy = linear_scale * dy
        vz = linear_scale * dz

        # Convert current and target orientations to roll, pitch, and yaw
        current_orientation = [self.current_position.orientation.x,
                               self.current_position.orientation.y,
                               self.current_position.orientation.z,
                               self.current_position.orientation.w]
        target_orientation = [self.target_position.orientation.x,
                              self.target_position.orientation.y,
                              self.target_position.orientation.z,
                              self.target_position.orientation.w]
        rospy.loginfo(f"Current orientation: {current_orientation}")
        rospy.loginfo(f"Target orientation: {target_orientation}")

        current_rpy = tf.transformations.euler_from_quaternion(current_orientation)
        target_rpy = tf.transformations.euler_from_quaternion(target_orientation)

        # Calculate angular differences and normalize for angular velocity
        roll_error = target_rpy[0] - current_rpy[0]
        pitch_error = target_rpy[1] - current_rpy[1]
        yaw_error = target_rpy[2] - current_rpy[2]

        # Normalize yaw, pitch, roll to [-pi, pi]
        roll_error = (roll_error + math.pi) % (2 * math.pi) - math.pi
        pitch_error = (pitch_error + math.pi) % (2 * math.pi) - math.pi
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        # Calculate angular velocities with scaling
        angular_scale_roll = min(self.max_angular_speed / abs(roll_error), 1.0) if roll_error != 0 else 0
        angular_scale_pitch = min(self.max_angular_speed / abs(pitch_error), 1.0) if pitch_error != 0 else 0
        angular_scale_yaw = min(self.max_angular_speed / abs(yaw_error), 1.0) if yaw_error != 0 else 0

        ax = angular_scale_roll * roll_error
        ay = angular_scale_pitch * pitch_error
        az = angular_scale_yaw * yaw_error

        ##--------------------------------------------------------------------------------------
        ##--------------------------------------------------------------------------------------


        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = vx
        twist_msg.linear.y = vy
        twist_msg.linear.z = vz
        twist_msg.angular.x = ax
        twist_msg.angular.y = ay
        twist_msg.angular.z = az

        self.vel_pub.publish(twist_msg)
        rospy.loginfo(f"Published Twist command: linear=({vx:.2f}, {vy:.2f}, {vz:.2f}), angular=({ax:.2f}, {ay:.2f}, {az:.2f})")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        velocity_profile_node = VelocityProfileNode()
        velocity_profile_node.run()
    except rospy.ROSInterruptException:
        pass
