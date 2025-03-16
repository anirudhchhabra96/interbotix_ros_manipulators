#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv
import tf.transformations as tf
from interbotix_xsarm_control.msg import CartesianCommand
from geometry_msgs.msg import Point, Twist, Pose
from threading import Lock
from scipy.interpolate import interp1d
import os
import time

class TrajectoryPublisher:
    def __init__(self, file_path, error_threshold=0.05):
        rospy.init_node("orbital_spinning_trajectory_publisher")
        self.pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)

        self.lock = Lock()
        self.actual_positions = []
        self.commanded_positions = []
        self.time_stamps = []
        self.start_time = None
        self.commanded_times = []
        self.actual_times = []
        self.waypoints = self.read_waypoints(file_path)
        self.actual_x = []
        self.actual_y = []
        self.actual_z = []
        self.commanded_x = []
        self.commanded_y = []
        self.commanded_z = []
        self.output_file = "tracking_record.txt"
        
        # Current actual pose, updated by subscriber
        self.current_actual_pose = None
        # Error threshold in meters
        self.error_threshold = error_threshold
        # Flag to indicate if we're recording data
        self.is_recording = False

        # Subscriber to read the actual end-effector position from the /ee_current_position topic
        rospy.Subscriber("/ee_current_position", Pose, self.ee_position_callback)

    def ee_position_callback(self, msg):
        # Extract position from Pose message
        actual_pose = (msg.position.x, msg.position.y, msg.position.z)
        self.current_actual_pose = actual_pose
        
        with self.lock:
            # Only record if we're in recording mode and start_time is set
            if self.is_recording and self.start_time is not None:
                current_time = rospy.Time.now().to_sec() - self.start_time
                self.actual_positions.append(actual_pose)
                self.actual_times.append(current_time)

    def euler_to_tf_quaternion(self, roll, pitch, yaw, axes='sxyz'):
        quat = tf.quaternion_from_euler(roll, pitch, yaw, axes=axes)
        return quat

    def normalize(self, v):
        return v / np.linalg.norm(v)

    def vector_to_quaternion(self, p):
        p = np.array(p)
        p_normalized = self.normalize(p)
        yaw = np.arctan2(p_normalized[1], p_normalized[0])
        pitch = np.arctan2(-p_normalized[2], np.linalg.norm(p_normalized[:2]))
        roll = 0.01
        quat = tf.quaternion_from_euler(roll, pitch, yaw)
        return quat

    def read_waypoints(self, file_path):
        waypoints = []
        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if len(row) == 6:
                    waypoints.append([float(val) for val in row])
        return waypoints
    
    def calculate_error(self, commanded_pos, actual_pos):
        """Calculate Euclidean distance between commanded and actual positions"""
        if actual_pos is None:
            return float('inf')
        return np.sqrt(sum((c - a) ** 2 for c, a in zip(commanded_pos, actual_pos)))

    def move_to_start_position(self, start_pos, timeout=30.0):
        """
        Move to the start position and wait until error is below threshold
        Returns True if position was reached, False if timed out
        """
        print(f"Moving to start position: {start_pos}")
        quaternion = self.vector_to_quaternion(start_pos)
        
        msg = CartesianCommand()
        msg.position = Point(start_pos[0], start_pos[1], start_pos[2])
        msg.velocity = Twist()
        msg.orientation.x, msg.orientation.y = quaternion[0], quaternion[1]
        msg.orientation.z, msg.orientation.w = quaternion[2], quaternion[3]
        
        # Publish the command repeatedly until threshold is reached or timeout
        rate = rospy.Rate(10)  # 10 Hz
        start_wait_time = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
            # Publish the target position
            self.pub.publish(msg)
            
            # Check if we've reached the target
            error = self.calculate_error(start_pos, self.current_actual_pose)
            current_time = rospy.Time.now().to_sec()
            
            # Print progress
            if self.current_actual_pose:
                print(f"Current position: {self.current_actual_pose}, Error: {error:.4f}m")
            
            # Check if we've reached the target or timed out
            if error < self.error_threshold:
                print(f"Start position reached! Error: {error:.4f}m")
                return True
            elif current_time - start_wait_time > timeout:
                print(f"Timeout reached while moving to start position. Final error: {error:.4f}m")
                return False
                
            rate.sleep()

    def publish_trajectory(self, start_idx=0, end_idx=None):
        end_idx = end_idx if end_idx is not None else len(self.waypoints)
        selected_waypoints = self.waypoints[start_idx:end_idx]
        
        if not selected_waypoints:
            print("No waypoints selected. Exiting.")
            return
        
        # Get the first waypoint to use as starting position
        first_wp = selected_waypoints[0]
        x, y, z = first_wp[0], first_wp[1], first_wp[2]
        # Apply offsets as in original code
        x, z = x - 0.2, z + 0.2
        start_pos = (x, y, z)
        
        # Move to start position and wait until error is below threshold
        reached_start = self.move_to_start_position(start_pos)
        if not reached_start:
            print("Failed to reach start position. Continuing anyway...")
        
        # Now that we're at the start position, begin recording and tracking
        self.is_recording = True
        with self.lock:
            self.start_time = rospy.Time.now().to_sec()
            
        print("Starting trajectory tracking and recording...")
        rate = rospy.Rate(1)  # 1 Hz (as in original code)
        
        for i, wp in enumerate(selected_waypoints):
            x, y, z, vx, vy, vz = wp
            x, z = x - 0.2, z + 0.2
            quaternion = self.vector_to_quaternion([x, y, z])

            msg = CartesianCommand()
            msg.position = Point(x, y, z)
            msg.velocity = Twist()
            msg.orientation.x, msg.orientation.y = quaternion[0], quaternion[1]
            msg.orientation.z, msg.orientation.w = quaternion[2], quaternion[3]

            with self.lock:
                self.commanded_positions.append((x, y, z))
                self.commanded_times.append(rospy.Time.now().to_sec() - self.start_time)

            self.pub.publish(msg)
            print(f"Published waypoint {i+1}/{len(selected_waypoints)}: ({x:.3f}, {y:.3f}, {z:.3f})")
            rate.sleep()
        
        # Sending a final stop command
        msg = CartesianCommand()
        msg.position = Point(x, y, z)
        msg.velocity = Twist()
        msg.velocity.linear.x = 0
        msg.velocity.linear.y = 0
        msg.velocity.linear.z = 0
        msg.orientation.x, msg.orientation.y = quaternion[0], quaternion[1]
        msg.orientation.z, msg.orientation.w = quaternion[2], quaternion[3]
        self.pub.publish(msg)
        print("Done with trajectory, press Ctrl+C to continue")

    def record_actual_trajectory(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

    def save_to_file(self):
        with open(self.output_file, 'w') as f:
            f.write("Time(s),Commanded X,Commanded Y,Commanded Z,Actual X,Actual Y,Actual Z\n")
            for i in range(len(self.commanded_times)):
                act_x = self.actual_x[i] if i < len(self.actual_x) else 0
                act_y = self.actual_y[i] if i < len(self.actual_y) else 0
                act_z = self.actual_z[i] if i < len(self.actual_z) else 0
                f.write(f"{self.commanded_times[i]},{self.commanded_x[i]},{self.commanded_y[i]},{self.commanded_z[i]},{act_x},{act_y},{act_z}\n")

    def plot_trajectory(self):
        with self.lock:
            if not self.commanded_positions or not self.actual_positions:
                print("No data to plot.")
                return

            cmd_x, cmd_y, cmd_z = zip(*self.commanded_positions)
            act_x, act_y, act_z = zip(*self.actual_positions)

            # Check if we have enough data points for interpolation
            if len(self.actual_times) < 2:
                print("Not enough actual trajectory data points for interpolation.")
                return
                
            interp_x = interp1d(self.actual_times, act_x, kind='linear', fill_value='extrapolate')
            interp_y = interp1d(self.actual_times, act_y, kind='linear', fill_value='extrapolate')
            interp_z = interp1d(self.actual_times, act_z, kind='linear', fill_value='extrapolate')
            act_x_interp = interp_x(self.commanded_times)
            act_y_interp = interp_y(self.commanded_times)
            act_z_interp = interp_z(self.commanded_times)

            self.actual_x = act_x_interp
            self.actual_y = act_y_interp
            self.actual_z = act_z_interp
            self.commanded_x = cmd_x
            self.commanded_y = cmd_y
            self.commanded_z = cmd_z

            # Plot position vs time for each axis
            fig, axs = plt.subplots(3, 1, figsize=(10, 8))
            axs[0].plot(self.commanded_times, cmd_x, 'b--', label='X Commanded')
            axs[0].plot(self.commanded_times, act_x_interp, 'b-', label='X Actual')
            axs[0].set_ylabel("X Position (m)")
            axs[0].legend()

            axs[1].plot(self.commanded_times, cmd_y, 'g--', label='Y Commanded')
            axs[1].plot(self.commanded_times, act_y_interp, 'g-', label='Y Actual')
            axs[1].set_ylabel("Y Position (m)")
            axs[1].legend()

            axs[2].plot(self.commanded_times, cmd_z, 'r--', label='Z Commanded')
            axs[2].plot(self.commanded_times, act_z_interp, 'r-', label='Z Actual')
            axs[2].set_ylabel("Z Position (m)")
            axs[2].set_xlabel("Time (s)")
            axs[2].legend()

            plt.tight_layout()
            plt.savefig("trajectory_tracking.png")
            
            # Plot 3D trajectory
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(cmd_x, cmd_y, cmd_z, 'b-', linewidth=2, label='Commanded')
            ax.plot(act_x_interp, act_y_interp, act_z_interp, 'r-', linewidth=2, label='Actual')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title('Commanded vs Actual 3D Trajectory')
            ax.legend()
            plt.savefig("trajectory_3d.png")
            
            # Calculate and plot error
            error_x = np.array(cmd_x) - act_x_interp
            error_y = np.array(cmd_y) - act_y_interp
            error_z = np.array(cmd_z) - act_z_interp
            error_magnitude = np.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            plt.figure(figsize=(10, 6))
            plt.plot(self.commanded_times, error_magnitude)
            plt.xlabel('Time (s)')
            plt.ylabel('Error Magnitude (m)')
            plt.title('Tracking Error')
            plt.grid(True)
            plt.savefig("tracking_error.png")
            
            print(f"Mean tracking error: {np.mean(error_magnitude):.4f} m")
            print(f"Max tracking error: {np.max(error_magnitude):.4f} m")
            print(f"Standard deviation of error: {np.std(error_magnitude):.4f} m")
            
            plt.show()

if __name__ == "__main__":
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))  # Get script's directory
        trajectory_filename = os.path.join(script_dir, "orbitdata_meo.txt")  # Construct absolute path
        
        # Create trajectory publisher with an error threshold of 3cm
        traj_pub = TrajectoryPublisher(trajectory_filename, error_threshold=0.03)
        
        # Publish trajectory and automatically move to start position first
        traj_pub.publish_trajectory(start_idx=44, end_idx=80)
        
        # Continue recording actual trajectory for a while
        traj_pub.record_actual_trajectory()
        
        # Plot and save results
        traj_pub.plot_trajectory()
        traj_pub.save_to_file()
    except rospy.ROSInterruptException:
        pass