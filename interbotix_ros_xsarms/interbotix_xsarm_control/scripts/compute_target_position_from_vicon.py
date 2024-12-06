from geometry_msgs.msg import Pose
import rospy
import tf_conversions

class DesiredPosePublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('desired_pose_publisher')

        # Publishers and subscribers
        self.target_pose = None
        self.current_pose = None
        self.robot_current_pose = None

        self.desired_pose_pub = rospy.Publisher('/ee_target_position', Pose, queue_size=10)
        rospy.Subscriber('/port', Pose, self.target_pose_callback)
        rospy.Subscriber('/module', Pose, self.current_pose_callback)
        rospy.Subscriber('/ee_current_position', Pose, self.robot_pose_callback)

    def target_pose_callback(self, msg):
        """Callback to store the target pose from the Vicon system."""
        self.target_pose = msg
        self.compute_and_publish_desired_pose()

    def current_pose_callback(self, msg):
        """Callback to store the current pose from the Vicon system."""
        self.current_pose = msg
        self.compute_and_publish_desired_pose()

    def robot_pose_callback(self, msg):
        """Callback to store the current pose of the robot in the robot frame."""
        self.robot_current_pose = msg
        self.compute_and_publish_desired_pose()

    def compute_relative_pose(self):
        """Compute the relative pose between current and target poses in the Vicon frame."""
        if self.current_pose is None or self.target_pose is None:
            return None

        relative_pose = Pose()

        # Compute relative position
        relative_pose.position.x = self.target_pose.position.x - self.current_pose.position.x
        relative_pose.position.y = self.target_pose.position.y - self.current_pose.position.y
        relative_pose.position.z = self.target_pose.position.z - self.current_pose.position.z

        # Compute relative orientation (quaternion difference)
        q1 = [self.current_pose.orientation.x, self.current_pose.orientation.y,
              self.current_pose.orientation.z, self.current_pose.orientation.w]
        q2 = [self.target_pose.orientation.x, self.target_pose.orientation.y,
              self.target_pose.orientation.z, self.target_pose.orientation.w]

        q_relative = tf_conversions.transformations.quaternion_multiply(
            q2, tf_conversions.transformations.quaternion_conjugate(q1)
        )
        relative_pose.orientation.x = q_relative[0]
        relative_pose.orientation.y = q_relative[1]
        relative_pose.orientation.z = q_relative[2]
        relative_pose.orientation.w = q_relative[3]

        return relative_pose

    def compute_and_publish_desired_pose(self):
        """Compute the desired pose and publish it if all data is available."""
        if self.robot_current_pose is None:
            return

        relative_pose = self.compute_relative_pose()
        if relative_pose is None:
            return

        desired_pose = Pose()

        # Add relative position
        desired_pose.position.x = self.robot_current_pose.position.x + relative_pose.position.x
        desired_pose.position.y = self.robot_current_pose.position.y + relative_pose.position.y
        desired_pose.position.z = self.robot_current_pose.position.z + relative_pose.position.z

        # Combine orientations (quaternion multiplication)
        q_robot = [self.robot_current_pose.orientation.x, self.robot_current_pose.orientation.y,
                   self.robot_current_pose.orientation.z, self.robot_current_pose.orientation.w]
        q_relative = [relative_pose.orientation.x, relative_pose.orientation.y,
                      relative_pose.orientation.z, relative_pose.orientation.w]

        q_combined = tf_conversions.transformations.quaternion_multiply(q_relative, q_robot)
        desired_pose.orientation.x = q_combined[0]
        desired_pose.orientation.y = q_combined[1]
        desired_pose.orientation.z = q_combined[2]
        desired_pose.orientation.w = q_combined[3]

        # Publish the desired pose
        self.desired_pose_pub.publish(desired_pose)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DesiredPosePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
