#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist

def circular_trajectory():
    rospy.init_node('ee_velocity_publisher', anonymous=True)
    vel_pub = rospy.Publisher('/desired_cartesian_velocity', Twist, queue_size=10)    
    rate = rospy.Rate(50)  # 50 Hz update rate
    
    r = 0.2  # Circle radius (meters)
    omega = 0.5  # Angular velocity (rad/s)
    height = 0.3  # Fixed height above the base frame
    start_time = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time  # Elapsed time
        
        # Compute velocities
        v_x = -r * omega * np.sin(omega * t)
        v_y = r * omega * np.cos(omega * t)
        v_z = 0  # No movement in the Z direction

        # Construct TwistStamped message
        vel_msg = Twist()
        vel_msg.linear.x = v_x 
        vel_msg.linear.y = v_y 
        vel_msg.linear.z = v_z 
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0  # Keep orientation fixed
        
        # Publish the velocity command
        vel_pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        circular_trajectory()
    except rospy.ROSInterruptException:
        pass
