import numpy as np
import tf.transformations as tf

# def euler_to_tf_quaternion(roll, pitch, yaw, axes='sxyz'):
#     """
#     Converts Euler angles to a quaternion matching the TF tree convention.

#     Parameters:
#         roll (float): Rotation around X-axis.
#         pitch (float): Rotation around Y-axis.
#         yaw (float): Rotation around Z-axis.
#         axes (str): Rotation order, default is 'sxyz'.
#         flip_yz (bool): If True, flips Y and Z components to match TF convention.
#         flip_xw (bool): If True, flips X and W components to match TF tree.

#     Returns:
#         list: Quaternion [x, y, z, w] matching the TF tree output.
#     """
#     quat = tf.quaternion_from_euler(roll, pitch, yaw, axes=axes)
    
#     return quat

# Test Euler angles:
roll = -0.239
pitch = -0.384
yaw = -2.777

# roll = 0.027
# pitch = -0.104
# yaw = -0.912

# roll = -0.082
# pitch = -0.054
# yaw = -2.669

quat_from_euler = tf.quaternion_from_euler(roll, pitch, yaw, axes='sxyz')

print("Quats:", quat_from_euler)
print("Euler:", tf.euler_from_quaternion(quat_from_euler, axes='sxyz'))
