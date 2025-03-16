#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import csv
import tf.transformations as tf
from interbotix_xsarm_control.msg import CartesianCommand
from geometry_msgs.msg import Point, Twist, Pose
from threading import Lock
from scipy.interpolate import interp1d
import os

class TrajectoryPublisher:
    def __init__(self, file_path):
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

        # Subscriber to read the actual end-effector position from the /ee_current_position topic
        rospy.Subscriber("/ee_current_position", Pose, self.ee_position_callback)

    def ee_position_callback(self, msg):
        # Extract position from Pose message
        actual_pose = (msg.position.x, msg.position.y, msg.position.z)
        with self.lock:
            if self.start_time is not None:
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

    def publish_trajectory(self, start_idx=0, end_idx=None):
        rate = rospy.Rate(1)  # 1 Hz
        end_idx = end_idx if end_idx is not None else len(self.waypoints)
        selected_waypoints = self.waypoints[start_idx:end_idx]

        for wp in selected_waypoints:
            x, y, z, vx, vy, vz = wp
            x, z = x - 0.2, z + 0.2
            quaternion = self.vector_to_quaternion([x, y, z])

            msg = CartesianCommand()
            msg.position = Point(x, y, z)
            msg.velocity = Twist()
            msg.orientation.x, msg.orientation.y = quaternion[0], quaternion[1]
            msg.orientation.z, msg.orientation.w = quaternion[2], quaternion[3]

            with self.lock:
                if self.start_time is None:
                    self.start_time = rospy.Time.now().to_sec()
                self.commanded_positions.append((x, y, z))
                self.commanded_times.append(rospy.Time.now().to_sec() - self.start_time)

            self.pub.publish(msg)
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
        print("Done, press Ctrl+C to continue")

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

            fig, axs = plt.subplots(3, 1, figsize=(10, 8))
            axs[0].plot(self.commanded_times, cmd_x, 'b--', label='X Commanded')
            axs[0].plot(self.commanded_times, act_x_interp, 'b-', label='X Actual')
            axs[0].set_ylabel("X Position")
            axs[0].legend()

            axs[1].plot(self.commanded_times, cmd_y, 'g--', label='Y Commanded')
            axs[1].plot(self.commanded_times, act_y_interp, 'g-', label='Y Actual')
            axs[1].set_ylabel("Y Position")
            axs[1].legend()

            axs[2].plot(self.commanded_times, cmd_z, 'r--', label='Z Commanded')
            axs[2].plot(self.commanded_times, act_z_interp, 'r-', label='Z Actual')
            axs[2].set_ylabel("Z Position")
            axs[2].set_xlabel("Time (s)")
            axs[2].legend()

            plt.tight_layout()
            plt.show()

if __name__ == "__main__":
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))  # Get script's directory
        trajectory_filename = os.path.join(script_dir, "orbitdata_meo.txt")  # Construct absolute path
        traj_pub = TrajectoryPublisher(trajectory_filename)
        traj_pub.publish_trajectory(start_idx=44, end_idx=80)
        traj_pub.record_actual_trajectory()
        traj_pub.plot_trajectory()
        traj_pub.save_to_file()
    except rospy.ROSInterruptException:
        pass
