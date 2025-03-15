#!/usr/bin/env python3
import rospy
import numpy as np
from interbotix_xsarm_control.msg import CartesianCommand
from geometry_msgs.msg import Point, Twist, PoseStamped, TransformStamped
import csv
import tf

def list_dimensions(list_):
    dimensions = []
    current = list_
    while isinstance(current, list):
        dimensions.append(len(current))
        if current:
            current = current[0]
        else:
            break
    return dimensions

def normalize(v):
    return v / np.linalg.norm(v)

def vector_to_quaternion(vector):
    # Normalize the direction vector
    vector = np.array(vector)
    vector_normalized = normalize(vector)

    # Local x-axis of the end effector (direction it should align with)
    x_local = np.array([1, 0, 0])

    # Compute the axis of rotation (cross product)
    axis = np.cross(x_local, vector_normalized)

    # If the vectors are parallel (i.e., axis is zero), no rotation is needed
    if np.linalg.norm(axis) < 1e-6:
        return np.array([1, 0, 0, 0])  # No rotation

    # Normalize the axis
    axis = normalize(axis)

    # Compute the angle of rotation (dot product)
    theta = np.arccos(np.clip(np.dot(x_local, vector_normalized), -1.0, 1.0))

    # Compute the quaternion components
    w = np.cos(theta / 2)
    x, y, z = axis * np.sin(theta / 2)

    return np.array([w, x, y, z])

def read_waypoints(file_path):
    waypoints = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if len(row) == 6:
                waypoints.append([float(val) for val in row])
    return waypoints

class TrajectoryPublisher:
    def __init__(self, waypoints, start_idx=0, end_idx=None, base_frame="base_link", target_link="desired_link"):
        rospy.init_node("orbital_spinning_trajectory_publisher")
        self.pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)
        self.rate = rospy.Rate(5)  # 10 Hz
        self.tf_listener = tf.TransformListener()
        self.base_frame = base_frame
        self.target_link = target_link

        self.waypoints = waypoints[start_idx:end_idx if end_idx is not None else len(waypoints)]

    def get_link_position(self):
        try:
            self.tf_listener.waitForTransform(self.base_frame, self.target_link, rospy.Time(0), rospy.Duration(1.0))
            (trans, _) = self.tf_listener.lookupTransform(self.base_frame, self.target_link, rospy.Time(0))
            return np.array(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to get link position from TF")
            return None

    def publish_trajectory(self):
        while not rospy.is_shutdown():
            link_position = self.get_link_position()
            if link_position is not None:
                break
            rospy.loginfo("Waiting for TF link position...")
            rospy.sleep(0.1)

        for i, wp in enumerate(self.waypoints):
            
            x, y, z, vx, vy, vz = wp
            
            # Set velocity to zero for the final waypoint
            if i == len(self.waypoints) - 1:
                vx, vy, vz = 0.0, 0.0, 0.0

            y = y + 0.2
            z = z + 0.2

            link_position = self.get_link_position()
            if link_position is None:
                rospy.logwarn("Skipping waypoint due to missing link position")
                continue

            direction_vector = np.array([x, y, z]) - link_position
            quaternion = vector_to_quaternion(direction_vector)
            
            msg = CartesianCommand()
            msg.position = Point(x, y, z)
            msg.velocity = Twist()
            msg.velocity.linear.x = 0
            msg.velocity.linear.y = 0
            msg.velocity.linear.z = 0
            msg.orientation.x = quaternion[1]
            msg.orientation.y = quaternion[2]
            msg.orientation.z = quaternion[3]
            msg.orientation.w = quaternion[0]
            
            rospy.loginfo(f"Publishing: Pos=({x}, {y}, {z}), Vel=({vx}, {vy}, {vz}), Quaternion=({quaternion})")
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        file_path = "orbitdata.txt"  # Change this to your file path
        waypoints = read_waypoints(file_path)
        start_idx = 65  # Set your desired start index
        end_idx = 85  # Set your desired end index, or None for full list
        base_frame = "mobile_wx250s/lower_forearm_link"  # Change as needed
        target_link = "mobile_wx250s/ee_arm_link"  # Change to the link you want

        traj_publisher = TrajectoryPublisher(waypoints, start_idx, end_idx, base_frame, target_link)
        traj_publisher.publish_trajectory()
    except rospy.ROSInterruptException:
        pass


