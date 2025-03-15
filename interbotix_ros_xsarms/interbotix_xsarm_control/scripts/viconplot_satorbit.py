#!/usr/bin/env python3

# import rospy
# import numpy as np
# import matplotlib.pyplot as plt
# import csv
# import tf.transformations as tf
# from interbotix_xsarm_control.msg import CartesianCommand
# from geometry_msgs.msg import Point, Twist, Pose
# from threading import Lock
# from scipy.interpolate import interp1d
# import tf

# class TrajectoryPublisher:
#     def __init__(self, file_path):
#         rospy.init_node("orbital_spinning_trajectory_publisher")
#         self.pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)
#         rospy.Subscriber("/satorbit", Pose, self.pose_callback)  # Actual trajectory feedback
        
#         self.lock = Lock()
#         self.actual_positions = []
#         self.commanded_positions = []
#         self.time_stamps = []
#         self.start_time = None
#         self.commanded_times = []
#         self.actual_times = []
#         self.waypoints = self.read_waypoints(file_path)

#         # Initialize TF listener
#         self.listener = tf.TransformListener()

#     def normalize(self, v):
#         return v / np.linalg.norm(v)

#     def vector_to_quaternion(self, p):
#         p = np.array(p)
#         p_normalized = self.normalize(p)
#         yaw = np.arctan2(p_normalized[1], p_normalized[0])
#         pitch = np.arctan2(-p_normalized[2], np.linalg.norm(p_normalized[:2]))
#         return tf.transformations.quaternion_from_euler(0, pitch, yaw)

#     def read_waypoints(self, file_path):
#         waypoints = []
#         with open(file_path, 'r') as file:
#             reader = csv.reader(file)
#             for row in reader:
#                 if len(row) == 6:
#                     waypoints.append([float(val) for val in row])
#         return waypoints

#     def publish_trajectory(self, start_idx=0, end_idx=None):
#         rate = rospy.Rate(1)  # 1 Hz
#         end_idx = end_idx if end_idx is not None else len(self.waypoints)
#         selected_waypoints = self.waypoints[start_idx:end_idx]

#         for wp in selected_waypoints:
#             x, y, z, vx, vy, vz = wp
#             x, z = x - 0.2, z + 0.2
#             quaternion = self.vector_to_quaternion([x, y, z])

#             msg = CartesianCommand()
#             msg.position = Point(x, y, z)
#             msg.velocity = Twist()
#             msg.orientation.x, msg.orientation.y = quaternion[0], quaternion[1]
#             msg.orientation.z, msg.orientation.w = quaternion[2], quaternion[3]

#             with self.lock:
#                 if self.start_time is None:
#                     self.start_time = rospy.Time.now().to_sec()
#                 self.commanded_positions.append((x, y, z))
#                 self.time_stamps.append(rospy.Time.now().to_sec() - self.start_time)
#                 self.commanded_times.append(rospy.Time.now().to_sec() - self.start_time)

#             rospy.loginfo(f"Publishing: Pos=({x}, {y}, {z})")
#             self.pub.publish(msg)
#             rate.sleep()

#     def pose_callback(self, msg):
#         with self.lock:
#             if self.start_time is None:
#                 return
#             current_time = rospy.Time.now().to_sec() - self.start_time

#             # Get the transformation from ee_arm_link to base_link
#             try:
#                 (trans, rot) = self.listener.lookupTransform('base_link', 'ee_arm_link', rospy.Time(0))
#                 # The Z position from the transformation (trans[2] is the Z value)
#                 z_position = trans[2]

#                 self.actual_positions.append((msg.position.x, msg.position.y, z_position))
#                 self.time_stamps.append(current_time)
#                 self.actual_times.append(current_time)
#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                 rospy.logwarn("TF exception in pose_callback.")

#     def plot_trajectory(self):
#         with self.lock:
#             if not self.commanded_positions or not self.actual_positions:
#                 print("No data to plot.")
#                 return

#             cmd_x, cmd_y, cmd_z = zip(*self.commanded_positions)
#             act_x, act_y, act_z = zip(*self.actual_positions)

#             # Interpolating actual positions to match commanded times
#             interp_x = interp1d(self.actual_times, act_x, kind='linear', fill_value='extrapolate')
#             interp_y = interp1d(self.actual_times, act_y, kind='linear', fill_value='extrapolate')
#             interp_z = interp1d(self.actual_times, act_z, kind='linear', fill_value='extrapolate')
#             act_x_interp = interp_x(self.commanded_times)
#             act_y_interp = interp_y(self.commanded_times)
#             act_z_interp = interp_z(self.commanded_times)

#             fig, axs = plt.subplots(3, 1, figsize=(10, 8))
#             axs[0].plot(self.commanded_times, cmd_x, 'b--', label='X Commanded')
#             axs[0].plot(self.commanded_times, act_x_interp, 'b-', label='X Actual')
#             axs[0].set_ylabel("X Position")
#             axs[0].legend()

#             axs[1].plot(self.commanded_times, cmd_y, 'g--', label='Y Commanded')
#             axs[1].plot(self.commanded_times, act_y_interp, 'g-', label='Y Actual')
#             axs[1].set_ylabel("Y Position")
#             axs[1].legend()

#             axs[2].plot(self.commanded_times, cmd_z, 'r--', label='Z Commanded')
#             axs[2].plot(self.commanded_times, act_z_interp, 'r-', label='Z Actual')
#             axs[2].set_ylabel("Z Position")
#             axs[2].set_xlabel("Time (s)")
#             axs[2].legend()

#             plt.tight_layout()
#             plt.show()
            

# if __name__ == "__main__":
#     try:
#         file_path = "orbitdata_meo.txt"
#         traj_pub = TrajectoryPublisher(file_path)
#         traj_pub.publish_trajectory(start_idx=41, end_idx=88)
#         traj_pub.plot_trajectory()
#     except rospy.ROSInterruptException:
#         pass

import rospy
import numpy as np
import matplotlib.pyplot as plt
import csv
import tf.transformations as tf
from interbotix_xsarm_control.msg import CartesianCommand
from geometry_msgs.msg import Point, Twist, Pose
from threading import Lock
from scipy.interpolate import interp1d

class TrajectoryPublisher:
    def __init__(self, file_path):
        rospy.init_node("orbital_spinning_trajectory_publisher")
        self.pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)
        rospy.Subscriber("/satorbit", Pose, self.pose_callback)  # Actual trajectory feedback
        
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
        self.output_file = "output_file.txt"


    def normalize(self, v):
        return v / np.linalg.norm(v)

    def vector_to_quaternion(self, p):
        p = np.array(p)
        p_normalized = self.normalize(p)
        yaw = np.arctan2(p_normalized[1], p_normalized[0])
        pitch = np.arctan2(-p_normalized[2], np.linalg.norm(p_normalized[:2]))
        return tf.quaternion_from_euler(0, pitch, yaw)

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
                self.time_stamps.append(rospy.Time.now().to_sec() - self.start_time)
                self.commanded_times.append(rospy.Time.now().to_sec() - self.start_time)

            rospy.loginfo(f"Publishing: Pos=({x}, {y}, {z})")
            self.pub.publish(msg)
            rate.sleep()

    def pose_callback(self, msg):
        with self.lock:
            if self.start_time is None:
                return
            current_time = rospy.Time.now().to_sec() - self.start_time
            self.actual_positions.append((msg.position.x+2, msg.position.y, msg.position.z-2))
            self.time_stamps.append(current_time)
            self.actual_times.append(current_time)
    def save_to_file(self):
        # Save the collected data to the file
        with open(self.output_file, 'w') as f:
            f.write("Time(s),Commanded X,Commanded Y,Commanded Z,Actual X,Actual Y,Actual Z\n")
            print(len(self.commanded_times))
            print(len(self.actual_x))
            print(len(self.commanded_x))
            for i in range(len(self.commanded_times)):
                # Ensure we don't go out of bounds (in case actual data is shorter)
                act_x = self.actual_x[i] if i < len(self.actual_x) else 0
                act_y = self.actual_y[i] if i < len(self.actual_y) else 0
                act_z = self.actual_z[i] if i < len(self.actual_z) else 0
                f.write(f"{self.commanded_times[i]},{self.commanded_x[i]},{self.commanded_y[i]},{self.commanded_z[i]},{act_x},{act_y},{act_z}\n")
                print(i)
    def plot_trajectory(self):
        with self.lock:
            if not self.commanded_positions or not self.actual_positions:
                print("No data to plot.")
                return

            cmd_x, cmd_y, cmd_z = zip(*self.commanded_positions)
            act_x, act_y, act_z = zip(*self.actual_positions)

            # Interpolating actual positions to match commanded times
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
        file_path = "orbitdata_meo.txt"
        traj_pub = TrajectoryPublisher(file_path)
        traj_pub.publish_trajectory(start_idx=41, end_idx=88)
        traj_pub.plot_trajectory()
        traj_pub.save_to_file()  # Save the collected data to the file
    except rospy.ROSInterruptException:
        pass

