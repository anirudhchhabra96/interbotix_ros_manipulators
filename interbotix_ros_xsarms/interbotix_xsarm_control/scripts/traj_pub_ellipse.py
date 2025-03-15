#!/usr/bin/env python3
import rospy
import numpy as np
from interbotix_xsarm_control.msg import CartesianCommand
from geometry_msgs.msg import Point, Quaternion, Twist
# import tf.transformations as tf
import tf
from tf.transformations import euler_from_quaternion
#!/usr/bin/env python3
import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3Stamped, Pose
from std_msgs.msg import Float64MultiArray
import geometry_msgs
from interbotix_xs_msgs.msg import JointGroupCommand  # Import the correct message type
import tf.transformations
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_vector3
import geometry_msgs.msg
import tf
import os
from tf2_msgs.msg import TFMessage
import tf.transformations as tf_transformations
from interbotix_xsarm_control.msg import CartesianCommand  # Import the new message type
from scipy.spatial.transform import Rotation as R

def generate_orbital_spinning_trajectory():
    rospy.init_node("orbital_spinning_trajectory_publisher")
    pub = rospy.Publisher("/desired_cartesian_command", CartesianCommand, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Define trajectory properties
    radius = 0.3  # Radius of the circular motion around the arm
    height = 0.3  # Fixed height above the base
    tilt_angle = np.deg2rad(0)  # Fixed tilt for natural movement
    spin_rate = np.deg2rad(0)  # Cube spinning speed (slow)

    # Orbital parameters
    omega = np.deg2rad(5)  # Orbital angular velocity (controls speed around the arm)
    T = 2 * np.pi / omega  # Time for one full orbit

    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time
        if t > T:  # Stop after one full loop
            print("one loop done")
            break
        
        # Compute circular trajectory around the arm's base
        theta = omega * t  # Angle around the arm
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        z = height  # Keep constant height

        # Compute velocity (numerical derivative)
        vx = -radius * omega * np.sin(theta)
        vy = radius * omega * np.cos(theta)
        vz = 0  # No vertical motion

        # Compute rotation to slowly spin the cube
        spin_angle = spin_rate * t  # Slow, continuous rotation
        orientation_quat = tf.transformations.quaternion_from_euler(tilt_angle, spin_angle, theta)  # Tilt + spin + orbit

        # Create and publish Cartesian command
        msg = CartesianCommand()
        msg.position = Point(x, y, z)
        # msg.orientation = Quaternion(*orientation_quat)  # Dynamic spinning orientation
        msg.orientation.x = orientation_quat[0]
        msg.orientation.y = orientation_quat[1]
        msg.orientation.z = orientation_quat[2]
        msg.orientation.w = orientation_quat[3]

        msg.velocity = Twist()
        msg.velocity.linear.x = vx
        msg.velocity.linear.y = vy
        msg.velocity.linear.z = vz
        print(msg.orientation)
        pub.publish(msg)
        rospy.loginfo(f"Published: Pos=({x:.3f}, {y:.3f}, {z:.3f})")

        rate.sleep()

if __name__ == "__main__":
    try:
        generate_orbital_spinning_trajectory()
    except rospy.ROSInterruptException:
        pass
