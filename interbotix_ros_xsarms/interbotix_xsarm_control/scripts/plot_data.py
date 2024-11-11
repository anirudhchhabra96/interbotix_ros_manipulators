#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt2
import numpy as np

# Define joint names in the order we want to plot
joint_names = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
num_joints = len(joint_names)

# Initialize data containers
x_data = []
y_data = [[] for _ in range(num_joints)]  # Create a list of lists for each joint's velocity data

def callback(msg):
    global x_data, y_data

    # Record the current time
    current_time = rospy.get_time()
    x_data.append(current_time)

    # Map the joint names to indices in the message
    joint_map = {name: i for i, name in enumerate(msg.name)}

    # Extract and store the velocity for each joint in the specified order
    for i, joint_name in enumerate(joint_names):
        if joint_name in joint_map:
            joint_index = joint_map[joint_name]
            velocity = msg.velocity[joint_index]
            y_data[i].append(velocity)
        else:
            y_data[i].append(0.0)  # Append 0 if the joint is not found

    # Keep only the last 100 data points for display
    if len(x_data) > 100:
        x_data.pop(0)
        for joint_velocities in y_data:
            joint_velocities.pop(0)

if __name__ == '__main__':
    rospy.init_node('plot_node')

    # Subscribe to the joint states topic
    topic_name = '/joint_states_sim'  # Replace with the actual topic name if needed
    rospy.Subscriber(topic_name, JointState, callback)

    # Initialize the plot
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    lines = [ax.plot([], [], label=joint_names[i])[0] for i in range(num_joints)] 
    ax.set_title("Joint Velocities Over Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (rad/s)")
    ax.legend(loc='upper right')  # Adjust location as needed

    # Main loop to update the plot
    while not rospy.is_shutdown():
        for i, line in enumerate(lines):
            line.set_data(x_data, y_data[i])  # Update each line with its joint's data
        
        # Adjust the plot limits
        ax.relim()
        ax.autoscale_view()
        plt.draw()
        plt.pause(0.1)
