#!/usr/bin/env python3

import rospy
import numpy as np
from interbotix_xsarm_control.msg import CartesianCommand
from geometry_msgs.msg import Point, Quaternion, Twist

def generate_smooth_trajectory():
    rospy.init_node("smooth_trajectory_publisher")
    pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)
    rate = rospy.Rate(50)  # 50 Hz for smooth control

    # Waypoints for the trajectory
    # Get current position as starting point (assume we need to read it first)
    # For this script, we'll create a brief pause to get current position
    rospy.sleep(1.0)  # Wait to ensure node is registered
    
    # In a real implementation, you'd get the current position from a topic
    # This is a placeholder - in practice you might want to:
    # 1. Subscribe to the robot's current state
    # 2. Wait for the first message
    # 3. Use that as the starting position
    # For now, we'll assume a starting position
    
    # Approximate starting position (would be replaced with actual reading)
    start_pos = [0.09, 0.0, 0.17]  # Example starting position
    
    # Define waypoints
    waypoints = [
        start_pos,                # Current position (placeholder)
        [0.2, -0.3, 0.2],         # First target
        [0.2, 0.3, 0.2]           # Final target
    ]
    
    # Timing parameters
    segment_durations = [5.0, 4.0]  # Time to travel between waypoints (seconds)
    
    # Use a valid, normalized quaternion for downward orientation
    # This quaternion represents a rotation that points the end-effector downward
    # For most common robot setups, this would be a 180-degree rotation around Y axis
    # downward_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # 180-degree rotation around Y axis
    downward_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # 180-degree rotation around Y axis
    
    # Alternative quaternions for different orientations if needed:
    # identity_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # No rotation
    # forward_orientation = Quaternion(0.0, 0.0, 0.7071, 0.7071)  # 90-degree rotation around Z
    
    # Start execution
    start_time = rospy.Time.now().to_sec()
    total_time = sum(segment_durations)
    
    # Main control loop
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec() - start_time
        
        # Check if trajectory is complete
        if current_time >= total_time:
            # Hold the final position with zero velocity
            final_position = waypoints[-1]
            msg = CartesianCommand()
            msg.position = Point(final_position[0], final_position[1], final_position[2])
            msg.orientation = downward_orientation
            msg.velocity = Twist()  # All zeros by default
            pub.publish(msg)
            
            rospy.loginfo(f"Trajectory complete. Holding position: ({final_position[0]:.3f}, {final_position[1]:.3f}, {final_position[2]:.3f})")
            rate.sleep()
            continue
        
        # Determine which segment we're in
        segment_idx = 0
        segment_time = current_time
        while segment_idx < len(segment_durations) and segment_time >= segment_durations[segment_idx]:
            segment_time -= segment_durations[segment_idx]
            segment_idx += 1
            
        # If we've gone through all segments but time remains, hold final position
        if segment_idx >= len(segment_durations):
            final_position = waypoints[-1]
            msg = CartesianCommand()
            msg.position = Point(final_position[0], final_position[1], final_position[2])
            msg.orientation = downward_orientation
            msg.velocity = Twist()  # All zeros by default
            pub.publish(msg)
            
            rospy.loginfo(f"Trajectory complete. Holding position: ({final_position[0]:.3f}, {final_position[1]:.3f}, {final_position[2]:.3f})")
            rate.sleep()
            continue
        
        # Calculate the normalized segment time (0 to 1)
        s = segment_time / segment_durations[segment_idx]
        
        # Apply smooth S-curve velocity profile using 5th order polynomial
        # s(t) = 6t^5 - 15t^4 + 10t^3
        # This gives zero velocity and acceleration at endpoints
        s_smooth = 6 * s**5 - 15 * s**4 + 10 * s**3
        
        # Calculate derivative of s_smooth for velocity scaling
        s_smooth_dot = 30 * s**4 - 60 * s**3 + 30 * s**2
        
        # Get the waypoints for the current segment
        start_point = waypoints[segment_idx]
        end_point = waypoints[segment_idx + 1]
        
        # Interpolate position
        x = start_point[0] + (end_point[0] - start_point[0]) * s_smooth
        y = start_point[1] + (end_point[1] - start_point[1]) * s_smooth
        z = start_point[2] + (end_point[2] - start_point[2]) * s_smooth
        
        # Calculate velocities
        vx = (end_point[0] - start_point[0]) * s_smooth_dot / segment_durations[segment_idx]
        vy = (end_point[1] - start_point[1]) * s_smooth_dot / segment_durations[segment_idx]
        vz = (end_point[2] - start_point[2]) * s_smooth_dot / segment_durations[segment_idx]
        
        # Create message
        msg = CartesianCommand()
        msg.position = Point(x, y, z)
        msg.orientation = downward_orientation
        
        # Set the velocity components
        msg.velocity = Twist()
        msg.velocity.linear.x = 0.0
        msg.velocity.linear.y = 0.0
        msg.velocity.linear.z = 0.0
        msg.velocity.angular.x = 0.0
        msg.velocity.angular.y = 0.0
        msg.velocity.angular.z = 0.0
        
        # Publish message
        pub.publish(msg)
        
        # Calculate progress percentage for the entire trajectory
        total_progress = (current_time / total_time) * 100
        
        rospy.loginfo(f"Published: Pos=({x:.3f}, {y:.3f}, {z:.3f}), Vel=({vx:.3f}, {vy:.3f}, {vz:.3f}), Progress={total_progress:.1f}%")
        
        rate.sleep()

if __name__ == "__main__":
    try:
        generate_smooth_trajectory()
    except rospy.ROSInterruptException:
        pass
# #!/usr/bin/env python3

# import rospy
# import numpy as np
# from interbotix_xsarm_control.msg import CartesianCommand
# from geometry_msgs.msg import Point, Quaternion, Twist

# def generate_tilted_semicircular_trajectory():
#     rospy.init_node("tilted_semicircular_trajectory_publisher")
#     pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)
#     rate = rospy.Rate(10)  # 10 Hz

#     # Define workspace limits
#     x_min, y_min, z_min = -0.4, -0.4, 0.1
#     x_max, y_max, z_max = 0.4, 0.4, 0.7

#     # Calculate center point within limits
#     x0 = (x_min + x_max) / 2  # Center X (0.0)
#     y0 = (y_min + y_max) / 2  # Center Y (0.0)
#     z0 = z_min + 0.1  # Start a bit above min Z (0.2)

#     # Calculate safe semi-axes to stay within limits
#     # Using 80% of available space to be safe
#     a = (x_max - x_min) * 0.4  # Semi-major axis (x-direction)
#     b = (y_max - y_min) * 0.4  # Semi-minor axis (y-direction)
    
#     # Define motion parameters - REDUCED TO 1/4 SPEED
#     omega = 0.2 * 0.25  # Angular velocity (rad/s) - reduced to 1/4
    
#     # Calculate safe z-velocity to reach max height in about 60 seconds
#     # Also reduced to 1/4 speed
#     z_range = z_max - z0 - 0.1  # Leave some margin at the top
#     vz = (z_range / 60.0) * 0.25  # Reach close to max height in 240 seconds (4x slower)
    
#     # Set the end-effector orientation to face downward
#     # This quaternion represents a 180-degree rotation around the X-axis
#     # which will make the end-effector point downward
#     downward_orientation = Quaternion(0.0, 0.0, 1.0, 0.0)  # 180-degree rotation around X axis

#     # Tilt angle for the ellipse (in radians)
#     tilt_angle = np.pi/3  # 30 degrees tilt
    
#     start_time = rospy.Time.now().to_sec()
    
#     # Define angle range for motion from -y to +y direction
#     start_angle = -np.pi/2  # Start from bottom (-y)
#     end_angle = np.pi/2     # End at top (+y)
#     angle_range = end_angle - start_angle  # Total angular distance to travel
    
#     # Time to complete one pass (from -y to +y)
#     pass_duration = angle_range / omega
    
#     while not rospy.is_shutdown():
#         t = rospy.Time.now().to_sec() - start_time  # Elapsed time
        
#         # Calculate current position in the motion cycle
#         cycle_time = t % pass_duration  # Time within current pass
#         progress = cycle_time / pass_duration  # Progress from 0 to 1
        
#         # Calculate current angle (from -π/2 to π/2)
#         angle = start_angle + progress * angle_range
        
#         # Reset Z position if we reach near the max height
#         current_z = z0 + vz * t
#         if current_z >= z_max - 0.1:
#             # Reset Z but keep the XY motion continuing
#             start_time = rospy.Time.now().to_sec() - (current_z - z0) / vz
#             t = rospy.Time.now().to_sec() - start_time
#             current_z = z0 + vz * t
        
#         # Calculate untilted elliptical coordinates
#         x_untilted = a * np.cos(angle)
#         y_untilted = b * np.sin(angle)
        
#         # Apply tilt transformation around X-axis (tilts the Y-Z plane)
#         # No change to x coordinate
#         x = x0 + x_untilted
#         y = y0 + y_untilted * np.cos(tilt_angle)
#         z = current_z + y_untilted * np.sin(tilt_angle)
        
#         # Calculate velocities with tilt
#         vx_untilted = -a * omega * np.sin(angle)
#         vy_untilted = b * omega * np.cos(angle)
        
#         vx = vx_untilted
#         vy = vy_untilted * np.cos(tilt_angle)
#         vz_from_tilt = vy_untilted * np.sin(tilt_angle)
        
#         # Combine vertical velocities
#         vz_total = vz + vz_from_tilt
        
#         # Ensure position stays within bounds (safety check)
#         x = np.clip(x, x_min + 0.05, x_max - 0.05)
#         y = np.clip(y, y_min + 0.05, y_max - 0.05)
#         z = np.clip(z, z_min + 0.05, z_max - 0.05)

#         # Create message
#         msg = CartesianCommand()
#         msg.position = Point(x, y, z)
#         msg.orientation = downward_orientation  # Use downward-facing orientation
        
#         # Set the velocity components
#         msg.velocity = Twist()
#         msg.velocity.linear.x = vx
#         msg.velocity.linear.y = vy
#         msg.velocity.linear.z = vz_total
#         msg.velocity.angular.x = 0.0
#         msg.velocity.angular.y = 0.0
#         msg.velocity.angular.z = 0.0  # No rotation

#         # Publish message
#         pub.publish(msg)
#         rospy.loginfo(f"Published: Pos=({x:.3f}, {y:.3f}, {z:.3f}), Vel=({vx:.3f}, {vy:.3f}, {vz_total:.3f}), Progress={progress:.2f}")

#         rate.sleep()

# if __name__ == "__main__":
#     try:
#         generate_tilted_semicircular_trajectory()
#     except rospy.ROSInterruptException:
#         pass
    
    
# #!/usr/bin/env python3

# import rospy
# import numpy as np
# from interbotix_xsarm_control.msg import CartesianCommand
# from geometry_msgs.msg import Point, Quaternion, Twist

# def generate_semicircular_trajectory():
#     rospy.init_node("semicircular_trajectory_publisher")
#     pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=5)
#     rate = rospy.Rate(10)  # 10 Hz

#     # Define workspace limits
#     x_min, y_min, z_min = -0.4, -0.4, 0.1
#     x_max, y_max, z_max = 0.4, 0.4, 0.7

#     # Calculate center point within limits
#     x0 = (x_min + x_max) / 2  # Center X (0.0)
#     y0 = (y_min + y_max) / 2  # Center Y (0.0)
#     z0 = z_min + 0.1  # Start a bit above min Z (0.2)

#     # Calculate safe semi-axes to stay within limits
#     # Using 80% of available space to be safe
#     a = (x_max - x_min) * 0.4  # Semi-major axis (x-direction)
#     b = (y_max - y_min) * 0.4  # Semi-minor axis (y-direction)
    
#     # Define motion parameters - REDUCED TO 1/4 SPEED
#     omega = 0.2 * 0.25  # Angular velocity (rad/s) - reduced to 1/4
    
#     # Calculate safe z-velocity to reach max height in about 60 seconds
#     # Also reduced to 1/4 speed
#     z_range = z_max - z0 - 0.1  # Leave some margin at the top
#     vz = (z_range / 60.0) * 0.25  # Reach close to max height in 240 seconds (4x slower)
    
#     # Set the end-effector orientation to face downward
#     # This quaternion represents a 180-degree rotation around the X-axis
#     # which will make the end-effector point downward
#     downward_orientation = Quaternion(1.0, 0.0, 0.0, 0.0)  # 180-degree rotation around X axis

#     start_time = rospy.Time.now().to_sec()
    
#     # New: Define angle range for motion from -y to +y direction
#     # Start from -π/2 (bottom of ellipse, -y direction) and move to π/2 (top of ellipse, +y direction)
#     start_angle = -np.pi/2  # Start from bottom (-y)
#     end_angle = np.pi/2     # End at top (+y)
#     angle_range = end_angle - start_angle  # Total angular distance to travel
    
#     # Time to complete one pass (from -y to +y)
#     pass_duration = angle_range / omega
    
#     while not rospy.is_shutdown():
#         t = rospy.Time.now().to_sec() - start_time  # Elapsed time
        
#         # Calculate current position in the motion cycle
#         cycle_time = t % pass_duration  # Time within current pass
#         progress = cycle_time / pass_duration  # Progress from 0 to 1
        
#         # Calculate current angle (from -π/2 to π/2)
#         angle = start_angle + progress * angle_range
        
#         # Reset Z position if we reach near the max height
#         current_z = z0 + vz * t
#         if current_z >= z_max - 0.1:
#             # Reset Z but keep the XY motion continuing
#             start_time = rospy.Time.now().to_sec() - (current_z - z0) / vz
#             t = rospy.Time.now().to_sec() - start_time
#             current_z = z0 + vz * t
        
#         # Elliptical motion
#         x = x0 + a * np.cos(angle)
#         y = y0 + b * np.sin(angle)
        
#         # Velocity for the half-ellipse
#         vx = -a * omega * np.sin(angle)
#         vy = b * omega * np.cos(angle)
        
#         z = current_z  # Linear motion along Z
        
#         # Ensure position stays within bounds (safety check)
#         x = np.clip(x, x_min + 0.05, x_max - 0.05)
#         y = np.clip(y, y_min + 0.05, y_max - 0.05)
#         z = np.clip(z, z_min + 0.05, z_max - 0.05)

#         # Create message
#         msg = CartesianCommand()
#         msg.position = Point(x, y, z)
#         msg.orientation = downward_orientation  # Use downward-facing orientation
        
#         # Set the velocity components
#         msg.velocity = Twist()
#         msg.velocity.linear.x = vx
#         msg.velocity.linear.y = vy
#         msg.velocity.linear.z = vz
#         msg.velocity.angular.x = 0.0
#         msg.velocity.angular.y = 0.0
#         msg.velocity.angular.z = 0.0  # No rotation

#         # Publish message
#         pub.publish(msg)
#         rospy.loginfo(f"Published: Pos=({x:.3f}, {y:.3f}, {z:.3f}), Vel=({vx:.3f}, {vy:.3f}, {vz:.3f}), Progress={progress:.2f}")

#         rate.sleep()

# if __name__ == "__main__":
#     try:
#         generate_semicircular_trajectory()
#     except rospy.ROSInterruptException:
#         pass