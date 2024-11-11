#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist

class VelocityComparisonPlotter:
    def __init__(self):
        rospy.init_node("velocity_comparison_plotter", anonymous=True)
        
        # Initialize storage for desired and actual velocities
        self.desired_velocity = Twist()
        self.actual_velocity = Twist()
        
        # Time and velocity history lists
        self.time_history = []
        self.desired_velocity_history = { "linear_x": [], "linear_y": [], "linear_z": [], 
                                          "angular_x": [], "angular_y": [], "angular_z": [] }
        self.actual_velocity_history = { "linear_x": [], "linear_y": [], "linear_z": [], 
                                         "angular_x": [], "angular_y": [], "angular_z": [] }
        
        # Subscribers
        rospy.Subscriber("/desired_cartesian_velocity", Twist, self.desired_velocity_callback)
        rospy.Subscriber("/end_effector_velocity", Twist, self.actual_velocity_callback)
        
        # Set up the plot
        self.fig, self.axs = plt.subplots(2, 3, figsize=(15, 10))
        
        self.initialize_plots()
        self.rate = rospy.Rate(10)  # 10 Hz update rate
    
    def initialize_plots(self):
        # Labels for each subplot
        labels = ["Linear X", "Linear Y", "Linear Z", "Angular X", "Angular Y", "Angular Z"]
        
        for i, ax in enumerate(self.axs.flat):
            ax.set_title(f"{labels[i]} Velocity")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Velocity (m/s or rad/s)")
            ax.grid()

    def desired_velocity_callback(self, msg):
        self.desired_velocity = msg
    
    def actual_velocity_callback(self, msg):
        self.actual_velocity = msg
    
    def update_plots(self):
        # Update time history
        current_time = rospy.get_time()
        self.time_history.append(current_time)

        # Append desired velocities
        self.desired_velocity_history["linear_x"].append(self.desired_velocity.linear.x)
        self.desired_velocity_history["linear_y"].append(self.desired_velocity.linear.y)
        self.desired_velocity_history["linear_z"].append(self.desired_velocity.linear.z)
        self.desired_velocity_history["angular_x"].append(self.desired_velocity.angular.x)
        self.desired_velocity_history["angular_y"].append(self.desired_velocity.angular.y)
        self.desired_velocity_history["angular_z"].append(self.desired_velocity.angular.z)

        # Append actual velocities
        self.actual_velocity_history["linear_x"].append(self.actual_velocity.linear.x)
        self.actual_velocity_history["linear_y"].append(self.actual_velocity.linear.y)
        self.actual_velocity_history["linear_z"].append(self.actual_velocity.linear.z)
        self.actual_velocity_history["angular_x"].append(self.actual_velocity.angular.x)
        self.actual_velocity_history["angular_y"].append(self.actual_velocity.angular.y)
        self.actual_velocity_history["angular_z"].append(self.actual_velocity.angular.z)

        # Update each subplot with the full history of desired and actual velocities
        labels = ["linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"]
        for i, ax in enumerate(self.axs.flat):
            ax.cla()  # Clear current plot
            ax.plot(self.time_history, self.desired_velocity_history[labels[i]], "b-", label="Desired")
            ax.plot(self.time_history, self.actual_velocity_history[labels[i]], "r-", label="Actual")
            ax.set_title(f"{labels[i].capitalize()} Velocity")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Velocity (m/s or rad/s)")
            ax.legend()
            ax.grid()

    def run(self):
        while not rospy.is_shutdown():
            self.update_plots()
            plt.draw()
            plt.pause(0.001)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        plotter = VelocityComparisonPlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
