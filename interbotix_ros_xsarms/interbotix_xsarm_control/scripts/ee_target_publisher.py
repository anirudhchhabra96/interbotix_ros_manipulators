#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import numpy as np

class EETargetPublisher:
    def __init__(self):
        rospy.init_node('ee_target_publisher', anonymous=True)

        # Publishers and Subscribers
        self.ee_target_pub = rospy.Publisher('/ee_target_position', Pose, queue_size=10)
        rospy.Subscriber('/top1', Pose, self.cube_top_pose_callback)
        rospy.Subscriber('/chaser_hex', Pose, self.arm_base_pose_callback)
        rospy.Subscriber('/ee_current_position', Pose, self.end_effector_pose_callback)

        # Variables
        self.cube_top_pose = None
        self.arm_base_pose = None
        self.end_effector_pose = None
        self.previous_cube_pose = None
        self.previous_end_effector_pose = None
        self.previous_time = rospy.Time.now()
        self.relative_velocity_threshold = 0.05

        # Cube dimensions (in meters)
        self.cube_side_length = 0.15

        # Offset for stopping in front of the hole (e.g., 0.1 m from the hole's opening)
        self.front_offset = 0.025

    def cube_top_pose_callback(self, msg):
        self.cube_top_pose = msg

    def arm_base_pose_callback(self, msg):
        self.arm_base_pose = msg

    def end_effector_pose_callback(self, msg):
        self.end_effector_pose = msg

    def compute_velocity(self, current_pose, previous_pose, time_delta):
        velocity = Twist()
        if current_pose is None or previous_pose is None or time_delta <= 0:
            return velocity

        velocity.linear.x = (current_pose.position.x - previous_pose.position.x) / time_delta
        velocity.linear.y = (current_pose.position.y - previous_pose.position.y) / time_delta
        velocity.linear.z = (current_pose.position.z - previous_pose.position.z) / time_delta

        # Angular velocity computation (simplified quaternion difference)
        q1 = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        q2 = [previous_pose.orientation.x, previous_pose.orientation.y, previous_pose.orientation.z, previous_pose.orientation.w]
        q_diff = tf.transformations.quaternion_multiply(q1, tf.transformations.quaternion_conjugate(q2))
        velocity.angular.x = q_diff[0] / time_delta
        velocity.angular.y = q_diff[1] / time_delta
        velocity.angular.z = q_diff[2] / time_delta

        return velocity

    def compute_relative_velocity(self):
        current_time = rospy.Time.now()
        time_delta = (current_time - self.previous_time).to_sec()

        cube_velocity = self.compute_velocity(self.cube_top_pose if self.cube_top_pose else None,
                                              self.previous_cube_pose if self.previous_cube_pose else None,
                                              time_delta)
        ee_velocity = self.compute_velocity(self.end_effector_pose if self.end_effector_pose else None,
                                            self.previous_end_effector_pose if self.previous_end_effector_pose else None,
                                            time_delta)

        self.previous_cube_pose = self.cube_top_pose
        self.previous_end_effector_pose = self.end_effector_pose
        self.previous_time = current_time

        if cube_velocity is None or ee_velocity is None:
            return None

        # Compute 6-DOF relative velocity
        relative_velocity = Twist()
        relative_velocity.linear.x = ee_velocity.linear.x - cube_velocity.linear.x
        relative_velocity.linear.y = ee_velocity.linear.y - cube_velocity.linear.y
        relative_velocity.linear.z = ee_velocity.linear.z - cube_velocity.linear.z

        relative_velocity.angular.x = ee_velocity.angular.x - cube_velocity.angular.x
        relative_velocity.angular.y = ee_velocity.angular.y - cube_velocity.angular.y
        relative_velocity.angular.z = ee_velocity.angular.z - cube_velocity.angular.z

        # Compute combined magnitude of relative velocity (translational and rotational)
        linear_magnitude = (relative_velocity.linear.x**2 + relative_velocity.linear.y**2 + relative_velocity.linear.z**2)**0.5
        angular_magnitude = (relative_velocity.angular.x**2 + relative_velocity.angular.y**2 + relative_velocity.angular.z**2)**0.5

        return linear_magnitude, angular_magnitude
    
    def compute_relative_orientation(self):
        if self.cube_top_pose is None or self.end_effector_pose is None:
            return None

        # Quaternion representing the relative orientation between the cube and end effector
        q_cube = [self.cube_top_pose.orientation.x, self.cube_top_pose.orientation.y,
                self.cube_top_pose.orientation.z, self.cube_top_pose.orientation.w]
        q_ee = [self.end_effector_pose.orientation.x, self.end_effector_pose.orientation.y,
                self.end_effector_pose.orientation.z, self.end_effector_pose.orientation.w]

        # Compute the quaternion difference
        q_diff = tf.transformations.quaternion_multiply(q_ee, tf.transformations.quaternion_conjugate(q_cube))

        # Compute the angular deviation (ignoring the scalar part)
        angular_deviation = np.arccos(np.clip(q_diff[3], -1.0, 1.0)) * 2  # Deviation in radians

        return angular_deviation

    def compute_target_pose(self):
        if self.cube_top_pose is None or self.arm_base_pose is None or self.end_effector_pose is None:
            return None

        # Compute the hole's position (center of the front face)
        hole_position = Pose()

        hole_position.position.x = 0.1061
        hole_position.position.y = 0
        hole_position.position.z = 0.1480
        hole_position.orientation.x = 0
        hole_position.orientation.y = 0
        hole_position.orientation.z = 0
        hole_position.orientation.w = 1

        q_target = [self.cube_top_pose.orientation.x, self.cube_top_pose.orientation.y,
                self.cube_top_pose.orientation.z, self.cube_top_pose.orientation.w]
        
        q_chaser = [self.arm_base_pose.orientation.x, self.arm_base_pose.orientation.y,
                self.arm_base_pose.orientation.z, self.arm_base_pose.orientation.w]
        
        q_diff = tf.transformations.quaternion_multiply(q_chaser, tf.transformations.quaternion_conjugate(q_target))

        # Compute the angular deviation (ignoring the scalar part)
        angular_diff1 = np.arccos(np.clip(q_diff[3], -1.0, 1.0)) * 2  # Deviation in radians


        if angular_diff1 < 0.5:

            half_side = self.cube_side_length / 2.0
            hole_position.position.x = self.cube_top_pose.position.x - half_side - self.front_offset
            hole_position.position.y = self.cube_top_pose.position.y
            hole_position.position.z = self.cube_top_pose.position.z - half_side
            hole_position.orientation = self.cube_top_pose.orientation

            # Transform end_effector_pose to inertial_frame using the arm_base_pose directly
            # hole_position.position.x -= self.arm_base_pose.position.x + self.end_effector_pose.position.x
            # hole_position.position.y -= self.arm_base_pose.position.y + self.end_effector_pose.position.y
            # hole_position.position.z -= self.arm_base_pose.position.z + self.end_effector_pose.position.z
            hole_position.position.x -= self.arm_base_pose.position.x
            hole_position.position.y -= self.arm_base_pose.position.y
            hole_position.position.z -= self.arm_base_pose.position.z
            
            hole_position.position.y += 0.0225
            # hole_position.position.y += 0.012 # this is manual calculation, visually observed offset
            hole_position.position.z += 0.01 # this is manual calculation, visually observed offset

            # Check relative velocity to decide whether to proceed into the hole
            relative_orientation = self.compute_relative_orientation()
            relative_velocity = self.compute_relative_velocity()
            if relative_velocity is not None:
                linear_magnitude, angular_magnitude = relative_velocity
                if linear_magnitude < 0.05 and angular_magnitude < 0.05 and relative_orientation < 0.05:
                    rospy.loginfo("Relative velocity low, moving into the hole.")
                    # Move into the hole
                    # hole_position.position.x = self.cube_top_pose.position.x - half_side
                    hole_position.position.x += self.front_offset + 0.015

        return hole_position

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            target_pose = self.compute_target_pose()
            if target_pose is not None:
                self.ee_target_pub.publish(target_pose)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = EETargetPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
