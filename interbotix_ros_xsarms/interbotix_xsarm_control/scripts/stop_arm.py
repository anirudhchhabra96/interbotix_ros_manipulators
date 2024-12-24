#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmControl:
    def __init__(self):
        rospy.init_node('arm_control', anonymous=True)
        
        # Publisher for sending the joint trajectory (for go-to-zero command)
        self.joint_trajectory_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        
        # Publisher to stop other operations (e.g., motion or commands)
        self.stop_pub = rospy.Publisher('/stop_all_operations', Bool, queue_size=10)

        # Set desired home position (zero position for the joints)
        self.zero_position = [0, -1.57, 1.57, 0, 0, 0]  # Adjust based on the robot's joint count
        
    def send_zero_position(self):
        # Create a JointTrajectory message
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]  # Replace with your robot's joint names
        
        # Create a JointTrajectoryPoint message to set all joints to zero
        point = JointTrajectoryPoint()
        point.positions = self.zero_position
        point.time_from_start = rospy.Duration(2.0)  # Adjust the time to reach zero position
        
        joint_trajectory.points = [point]
        
        # Send the "go to zero" command
        rospy.loginfo("Sending 'go to zero' command to the arm.")
        self.joint_trajectory_pub.publish(joint_trajectory)

        # Optionally stop any other ongoing operations by sending a stop signal
        rospy.loginfo("Stopping all other operations.")
        self.stop_pub.publish(True)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # For this example, we'll call the 'go to zero' command
            self.send_zero_position()
            rate.sleep()

if __name__ == "__main__":
    try:
        arm_control = ArmControl()
        arm_control.run()
    except rospy.ROSInterruptException:
        pass
