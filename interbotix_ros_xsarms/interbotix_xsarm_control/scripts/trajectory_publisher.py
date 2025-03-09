#!/usr/bin/env python3
import rospy
import numpy as np
from interbotix_xsarm_control.msg import CartesianCommand  # Import your custom message
from geometry_msgs.msg import Point, Quaternion, Twist

def generate_circular_trajectory2():
    rospy.init_node("test_cartesian_trajectory_publisher2")
    pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Define trajectory parameters
    x0, y0, z0 = 0.2, 0.0, 0.2  # Circle center
    radius = 0.1  # Circle radius
    omega = 0.2  # Angular velocity (rad/s)
    vz = 0.01  # Constant Z velocity

    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time  # Elapsed time

        # Compute desired position (circular motion)
        x = x0 + radius * np.cos(omega * t)
        y = y0 + radius * np.sin(omega * t)
        z = z0 + vz * t  # Linear motion along Z

        # Compute desired velocity
        vx = -radius * omega * np.sin(omega * t)
        vy = radius * omega * np.cos(omega * t)
        vz = vz  # Constant Z velocity

        # Create message
        msg = CartesianCommand()
        msg.position = Point(x, y, z)
        msg.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Keep orientation fixed
        msg.velocity.linear.x = vx
        msg.velocity.linear.y = vy
        msg.velocity.linear.z = vz
        msg.velocity.angular.x = 0.0
        msg.velocity.angular.y = 0.0
        msg.velocity.angular.z = 0.0  # No rotation

        # Publish message
        pub.publish(msg)
        rospy.loginfo(f"Published: Pos=({x:.3f}, {y:.3f}, {z:.3f}), Vel=({vx:.3f}, {vy:.3f}, {vz:.3f})")

        rate.sleep()

if __name__ == "__main__":
    try:
        generate_circular_trajectory2()
    except rospy.ROSInterruptException:
        pass