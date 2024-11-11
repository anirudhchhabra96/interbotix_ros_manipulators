#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class jointstates_sim:
    def __init__(self):
        # Initialize node
        rospy.init_node('velocity_estimator')
        
        # Set up subscribers and publishers
        self.joint_state_sub = rospy.Subscriber('/mobile_wx250s/joint_states', JointState, self.joint_state_callback)
        self.velocity_pub = rospy.Publisher('/joint_states_sim', JointState, queue_size=10)
        
        # Store previous joint states to calculate velocities
        self.prev_positions = None
        self.prev_time = None

    def joint_state_callback(self, msg):
        current_time = rospy.Time.now()
        
        if self.prev_positions is None:
            # First callback, just store the initial values
            self.prev_positions = msg.position
            self.prev_time = current_time
            return
        
        # Calculate time difference
        dt = (current_time - self.prev_time).to_sec()
        
        # Estimate joint velocities
        velocities = [(curr_pos - prev_pos) / dt for curr_pos, prev_pos in zip(msg.position, self.prev_positions)]
        
        # Create new JointState message with estimated velocities
        velocity_msg = JointState()
        velocity_msg.header = Header()
        velocity_msg.header.stamp = current_time
        velocity_msg.name = msg.name
        velocity_msg.position = msg.position  # Keep the positions the same
        velocity_msg.velocity = velocities  # Add the estimated velocities

        # Publish the message
        self.velocity_pub.publish(velocity_msg)
        
        # Update previous values
        self.prev_positions = msg.position
        self.prev_time = current_time

if __name__ == '__main__':
    try:
        jointstates_sim()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
