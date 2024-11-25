#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String


# This is where robot communicate


class Community:
    def __init__(self):
        # Dynamically fetch parameters for the robot
        self.robot_name = rospy.get_param('~robot_name', 'default_robot')
        self.robot_type = rospy.get_param('~robot_type', 'default_type')

        # Robot states: Running, Waiting, Leaving
        self.states = ["Running", "Waiting", "Leaving"]
        self.current_state = "Running"

        # Initialize the ROS node
        rospy.init_node(self.robot_name, anonymous=False)

        # Initialization
        self.publisher = rospy.Publisher('robot_info', String, queue_size=10)
        self.subscriber = rospy.Subscriber('robot_info', String, self.callback)

        # Initialize rate (10 Hz)
        self.rate = rospy.Rate(10)
        rospy.loginfo(f"{self.robot_type} ({self.robot_name}) initialized in state: {self.current_state}")

    def callback(self, data):
        """Callback function to process messages from other robots."""
        rospy.loginfo(f"Received message: {data.data}")

        # Process the message
        if self.robot_name not in data.data:  # Avoid self-response
            rospy.loginfo("Message is from another robot!")

    def publish_message(self):
        """Publishes the robot's type, name, and state."""
        message = f"Robot Type: {self.robot_type}, Robot Name: {self.robot_name}, State: {self.current_state}"
        rospy.loginfo(f"Publishing: {message}")
        self.publisher.publish(message)

    def run(self):
        """Main loop to publish messages."""
        while not rospy.is_shutdown():
            self.publish_message()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        community = Community()
        community.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
