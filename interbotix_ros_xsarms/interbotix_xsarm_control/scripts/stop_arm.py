#!/usr/bin/env python3
import rospy
from interbotix_xs_msgs.msg import JointGroupCommand
import subprocess

class ArmControl:
    def __init__(self):
        rospy.init_node('arm_control', anonymous=True)
        
        # Publisher for sending joint commands
        self.joint_group_pub = rospy.Publisher(
            '/mobile_wx250s/commands/joint_group', JointGroupCommand, queue_size=10
        )
        
        # Define the zero position for the arm
        self.zero_position = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # Radians for each joint
        
        # Joint group name (specific to the robot)
        self.joint_group_name = 'arm'  # Replace with your robot's group name if different

    def send_zero_position(self):
        # Create a JointGroupCommand message
        command = JointGroupCommand()
        command.name = self.joint_group_name
        command.cmd = self.zero_position.tolist()
        
        # Publish the command to set the arm to the zero position
        rospy.loginfo("Sending 'go to zero' command to the arm.")
        self.joint_group_pub.publish(command)

    def run(self):
        # Call the "go to zero" command once when the node is started
        rospy.sleep(1)  # Allow time for connections to establish
        self.send_zero_position()
        rospy.spin()

if __name__ == "__main__":
    try:
        arm_control = ArmControl()
        # subprocess.call(["rosnode", "kill", "mobile_wx250s/velocity_profile_node"])
        # subprocess.call(["rosnode", "kill", "mobile_wx250s/desired_pose_publisher"])
        # subprocess.call(["rosnode", "kill", "mobile_wx250s/ee_target_publisher"])
        # subprocess.call(["rosnode", "kill", "mobile_wx250s/ik_velocity_control"])
        # subprocess.call(["rosnode","kill","mobile_wx250s/ik_velocity_control"])

        arm_control.run()
    except rospy.ROSInterruptException:
        pass
