#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
import matplotlib.ticker as ticker

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
        self.fig, self.axs = plt.subplots(3,2, figsize=(13, 15))
        
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

        # Clear and update each subplot explicitly
        self.axs[0, 0].cla()
        self.axs[0, 0].plot(self.time_history, self.desired_velocity_history["linear_x"], "b-", label="Desired")
        self.axs[0, 0].plot(self.time_history, self.actual_velocity_history["linear_x"], "r-", label="Actual")
        # self.axs[0, 0].set_title("Linear X Velocity")
        self.axs[0, 0].set_xlabel("Time (s)")
        self.axs[0, 0].set_ylabel("x-axis Linear Velocity (m/s)")
        self.axs[0, 0].legend()
        self.axs[0, 0].grid()

        self.axs[0, 1].cla()
        self.axs[0, 1].plot(self.time_history, self.desired_velocity_history["angular_x"], "b-", label="Desired")
        self.axs[0, 1].plot(self.time_history, self.actual_velocity_history["angular_x"], "r-", label="Actual")
        # self.axs[1, 0].set_title("Angular X Velocity")
        self.axs[0, 1].set_xlabel("Time (s)")
        self.axs[0, 1].set_ylabel("x-axis Angular Velocity (rad/s)")
        self.axs[0, 1].legend()
        self.axs[0, 1].grid()





        self.axs[1, 0].cla()
        self.axs[1, 0].plot(self.time_history, self.desired_velocity_history["linear_y"], "b-", label="Desired")
        self.axs[1, 0].plot(self.time_history, self.actual_velocity_history["linear_y"], "r-", label="Actual")
        # self.axs[0, 1].set_title("Linear Y Velocity")
        self.axs[1, 0].set_xlabel("Time (s)")
        self.axs[1, 0].set_ylabel("y-axis Linear Velocity (m/s)")
        self.axs[1, 0].legend()
        self.axs[1, 0].grid()

        self.axs[1, 1].cla()
        self.axs[1, 1].plot(self.time_history, self.desired_velocity_history["angular_y"], "b-", label="Desired")
        self.axs[1, 1].plot(self.time_history, self.actual_velocity_history["angular_y"], "r-", label="Actual")
        # self.axs[1, 1].set_title("Angular Y Velocity")
        self.axs[1, 1].set_xlabel("Time (s)")
        self.axs[1, 1].set_ylabel("y-axis Angular Velocity (rad/s)")
        self.axs[1, 1].legend()
        self.axs[1, 1].grid()



        self.axs[2, 0].cla()
        self.axs[2, 0].plot(self.time_history, self.desired_velocity_history["linear_z"], "b-", label="Desired")
        self.axs[2, 0].plot(self.time_history, self.actual_velocity_history["linear_z"], "r-", label="Actual")
        # self.axs[0, 2].set_title("Linear Z Velocity")
        self.axs[2, 0].set_xlabel("Time (s)")
        self.axs[2, 0].set_ylabel("z-axis Linear Velocity (m/s)")
        self.axs[2, 0].legend()
        self.axs[2, 0].grid()

        self.axs[2, 1].cla()
        self.axs[2, 1].plot(self.time_history, self.desired_velocity_history["angular_z"], "b-", label="Desired")
        self.axs[2, 1].plot(self.time_history, self.actual_velocity_history["angular_z"], "r-", label="Actual")
        # self.axs[1, 2].set_title("Angular Z Velocity")
        self.axs[2, 1].set_xlabel("Time (s)")
        self.axs[2, 1].set_ylabel("z-axis Angular Velocity (rad/s)")
        self.axs[2, 1].legend()
        self.axs[2, 1].grid()
        
        self.axs[0, 0].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f')) # 2 decimal places
        self.axs[0, 1].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
        self.axs[1, 0].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
        self.axs[1, 1].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
        self.axs[2, 0].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))
        self.axs[2, 1].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.2f'))

    # def update_plots(self):
    #     # Update time history
    #     current_time = rospy.get_time()
    #     self.time_history.append(current_time)

    #     # Append desired velocities
    #     self.desired_velocity_history["linear_x"].append(self.desired_velocity.linear.x)
    #     self.desired_velocity_history["linear_y"].append(self.desired_velocity.linear.y)
    #     self.desired_velocity_history["linear_z"].append(self.desired_velocity.linear.z)
    #     self.desired_velocity_history["angular_x"].append(self.desired_velocity.angular.x)
    #     self.desired_velocity_history["angular_y"].append(self.desired_velocity.angular.y)
    #     self.desired_velocity_history["angular_z"].append(self.desired_velocity.angular.z)

    #     # Append actual velocities
    #     self.actual_velocity_history["linear_x"].append(self.actual_velocity.linear.x)
    #     self.actual_velocity_history["linear_y"].append(self.actual_velocity.linear.y)
    #     self.actual_velocity_history["linear_z"].append(self.actual_velocity.linear.z)
    #     self.actual_velocity_history["angular_x"].append(self.actual_velocity.angular.x)
    #     self.actual_velocity_history["angular_y"].append(self.actual_velocity.angular.y)
    #     self.actual_velocity_history["angular_z"].append(self.actual_velocity.angular.z)

    #     # Update each subplot with the full history of desired and actual velocities
    #     labels = ["linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"]
    #     for i, ax in enumerate(self.axs.flat):
    #         ax.cla()  # Clear current plot
    #         ax.plot(self.time_history, self.desired_velocity_history[labels[i]], "b-", label="Desired")
    #         ax.plot(self.time_history, self.actual_velocity_history[labels[i]], "r-", label="Actual")
    #         ax.set_title(f"{labels[i].capitalize()} Velocity")
    #         ax.set_xlabel("Time (s)")
    #         ax.set_ylabel("Velocity (m/s or rad/s)")
    #         ax.legend()
    #         ax.grid()

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
