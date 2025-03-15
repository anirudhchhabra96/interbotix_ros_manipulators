#!/usr/bin/env python3

import rospy
import numpy as np
from interbotix_xsarm_control.msg import CartesianCommand
from geometry_msgs.msg import Point, Twist
import csv
import tf.transformations as tf

def normalize(v):
    return v / np.linalg.norm(v)



def vector_to_quaternion(p):
    p = np.array(p)
    p_normalized = normalize(p)

    # Compute yaw (rotation about the z-axis)
    yaw = np.arctan2(p_normalized[1], p_normalized[0])  # atan2(y, x)
    
    # Compute pitch (rotation about the y-axis)
    pitch = np.arctan2(-p_normalized[2], np.linalg.norm(p_normalized[:2]))  # atan2(-z, sqrt(x^2 + y^2))

    # Convert Euler angles to quaternion (assuming roll = 0)
    quaternion = tf.quaternion_from_euler(0, pitch, yaw)

    return quaternion

# def vector_to_quaternion(p):
#     # Normalize the position vector
#     p = np.array(p)
#     p_normalized = normalize(p)

#     # Local x-axis of the end effector (direction it should align with)
#     x_local = np.array([1, 0, 0])

#     # Compute the axis of rotation (cross product)
#     axis = np.cross(x_local, p_normalized)

#     # If the vectors are parallel (i.e., axis is zero), no rotation is needed
#     if np.linalg.norm(axis) < 1e-6:
#         return np.array([1, 0, 0, 0])  # No rotation

#     # Normalize the axis
#     axis = normalize(axis)

#     # Compute the angle of rotation (dot product)
#     theta = np.arccos(np.dot(x_local, p_normalized))

#     # Compute the quaternion components
#     w = np.cos(theta / 2)
#     x, y, z = axis * np.sin(theta / 2)

#     return np.array([w, x, y, z])

def read_waypoints(file_path):
    waypoints = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if len(row) == 6:
                waypoints.append([float(val) for val in row])
    return waypoints

def publish_trajectory(waypoints, start_idx=0, end_idx=None):
    rospy.init_node("orbital_spinning_trajectory_publisher")
    pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)
    rate = rospy.Rate(1)  # 10 Hz
    end_idx = end_idx if end_idx is not None else len(waypoints)
    selected_waypoints = waypoints[start_idx:end_idx]
    print(list_dimensions(selected_waypoints))
    for wp in selected_waypoints:
        x, y, z, vx, vy, vz = wp
        x, z  = x - 0.2, z + 0.3
        # y = y + 0.2
        # z = z + 0.3
        
        quaternion = vector_to_quaternion([x, y, z])
        
        msg = CartesianCommand()
        msg.position = Point(x, y, z)
        msg.velocity = Twist()
        msg.velocity.linear.x = 0
        msg.velocity.linear.y = 0
        msg.velocity.linear.z = 0
        # msg.orientation.x = quaternion[1]
        # msg.orientation.y = quaternion[2]
        # msg.orientation.z = quaternion[3]
        # msg.orientation.w = quaternion[0]
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
            
        rospy.loginfo(f"Publishing: Pos=({x}, {y}, {z}), Vel=({vx}, {vy}, {vz}), Quaternion=({quaternion})")
        pub.publish(msg)
        rate.sleep()
    
    # while True:
    # msg = CartesianCommand()
    # msg.position = Point(x, y, z)
    # msg.velocity = Twist()
    msg.velocity.linear.x = 0
    msg.velocity.linear.y = 0
    msg.velocity.linear.z = 0
    # msg.orientation.x = quaternion[1]
    # msg.orientation.y = quaternion[2]
    # msg.orientation.z = quaternion[3]
    # msg.orientation.w = quaternion[0]
    pub.publish(msg)

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

if __name__ == "__main__":
    try:
        file_path = "orbitdata_meo.txt"  # Change this to your file path
        waypoints = read_waypoints(file_path)
        # print(list_dimensions(waypoints))
        start_idx = 41  # Set your desired start index
        end_idx = 93  # Set your desired end index, or None for full list
        publish_trajectory(waypoints, start_idx, end_idx)

        # publish_trajectory(waypoints)
    except rospy.ROSInterruptException:
        pass


# -0.0564609285099567, 0.124265722733904, 0.0233155012045414
# 