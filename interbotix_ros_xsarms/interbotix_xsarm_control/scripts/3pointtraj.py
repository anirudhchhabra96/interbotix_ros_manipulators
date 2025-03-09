#!/usr/bin/env python3
import rospy
import numpy as np
from interbotix_xsarm_control.msg import CartesianCommand
from geometry_msgs.msg import Point, Quaternion, Twist
import time
from sensor_msgs.msg import JointState
import math
angle1 = 0.0

def joint_state_callback(msg):
    
    # Assuming angle1 is the first joint in the message
    angle1 = msg.position[0]

def generate_three_point_trajectory():
    rospy.init_node("three_point_trajectory_publisher")
    pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    rospy.Subscriber("mobile_wx250s/joint_states", JointState, joint_state_callback)

    # Define waypoints
    waypoints = [
        np.array([0.2, -0.2, 0.3]),  # Start
        np.array([0.5,  0.0, 0.3]),  # Middle
        np.array([0.2,  0.2, 0.3])   # End
    ]

    move_time = 10.0  # Time to move between waypoints (seconds)
    num_steps = int(move_time * 10)  # 10 Hz updates

    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i + 1]

        for step in range(num_steps):
            alpha = step / float(num_steps)  # Linear interpolation factor
            pos = (1 - alpha) * start + alpha * end  # Interpolated position

            # Compute velocity as difference between current and next step
            if step < num_steps - 1:
                next_pos = (1 - (alpha + 1.0/num_steps)) * start + (alpha + 1.0/num_steps) * end
                velocity = (next_pos - pos) * 10  # Scale to match update rate
            else:
                velocity = np.zeros(3)  # Stop at the last step
            qx = 0.0
            qy = 0.0
            qz = math.sin(angle1 / 2.0)
            qw = math.cos(angle1 / 2.0)
            # Create Cartesian command message
            msg = CartesianCommand()
            msg.position = Point(pos[0], pos[1], pos[2])
            msg.orientation = Quaternion(qx,qy,qz,qw)  # Default orientation
            msg.velocity = Twist()
            msg.velocity.linear.x = velocity[0]
            msg.velocity.linear.y = velocity[1]
            msg.velocity.linear.z = velocity[2]

            pub.publish(msg)
            rospy.loginfo(f"Moving to: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

            rate.sleep()
    # Hold at the last waypoint
    rospy.loginfo("Holding position at the final point.")
    final_pos = waypoints[-1]
    hold_start_time = rospy.Time.now().to_sec()
    while True:
        msg = CartesianCommand()
        msg.position = Point(final_pos[0], final_pos[1], final_pos[2])
        msg.orientation = Quaternion(qx,qy,qz,qw)  # Keep same orientation
        msg.velocity = Twist()  # No movement (hold position)

        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Completed 3-point trajectory.")

if __name__ == "__main__":
    try:
        generate_three_point_trajectory()
    except rospy.ROSInterruptException:
        pass
