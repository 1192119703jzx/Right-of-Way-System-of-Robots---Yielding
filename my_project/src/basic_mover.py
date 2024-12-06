#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point

class BasicMover:
    def __init__(self):
        # Retrieve robot namespace and topics from parameters
        self.robot_namespace = rospy.get_param('~robot_namespace', '')
        odom_topic = rospy.get_param('~odom_topic', f"{self.robot_namespace}/my_odom")
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', f"{self.robot_namespace}/cmd_vel")
        
        # Log the namespace and topics
        rospy.loginfo(f"BasicMover initialized for namespace: {self.robot_namespace}")
        rospy.loginfo(f"Subscribing to: {odom_topic}")
        rospy.loginfo(f"Publishing to: {cmd_vel_topic}")
        
        # Publisher and Subscriber
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber(odom_topic, Point, self.odom_cb)


        # Movement control variables
        self.target_x = 1.0
        self.current_x = 0.0

    def odom_cb(self, msg):
        # Update current position and log it
        self.current_x = msg.x
        rospy.loginfo(f"[{self.robot_namespace}] Current X: {self.current_x}")
        self.move_to_target()

    def move_to_target(self):
        # Publish a movement command
        twist = Twist()
        if self.current_x < self.target_x:
            twist.linear.x = 0.2
        else:
            twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    BasicMover()
    rospy.spin()
