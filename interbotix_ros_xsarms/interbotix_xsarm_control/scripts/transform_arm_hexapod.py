#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def publish_static_transform():
    rospy.init_node("hexapod_arm_tf_publisher")
    br = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "platform_link"
    static_transformStamped.child_frame_id = "world"
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 17.0/1000  # Set your actual offset
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    br.sendTransform(static_transformStamped)
    rospy.spin()

if __name__ == "__main__":
    publish_static_transform()
