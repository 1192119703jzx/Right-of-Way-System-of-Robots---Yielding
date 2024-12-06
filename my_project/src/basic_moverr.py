#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point

class BasicMoverRafael:
    def __init__(self):
        # Namespace specific to Rafael
        self.robot_namespace = 'rafael'
        self.robot_name = 'rafael'

        # Rafael's specific topics
        odom_topic = f"/{self.robot_namespace}/my_odom"
        cmd_vel_topic = f"/{self.robot_namespace}/cmd_vel"

        # Log the topics being used
        rospy.loginfo(f"BasicMoverRafael initialized for namespace: {self.robot_namespace}")
        rospy.loginfo(f"Subscribing to: {odom_topic}")
        rospy.loginfo(f"Publishing to: {cmd_vel_topic}")

        # Publisher and Subscriber
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber(odom_topic, Point, self.odom_cb)

        # Movement control variables
        self.target_distance = 2.0  # Move until a total distance of 2.0 meters is reached
        self.current_distance = 0.0
        self.current_yaw = 0.0

    def odom_cb(self, msg):
        """
        Callback for processed odometry data.
        Updates the distance traveled and yaw, and determines whether to keep moving.
        """
        self.current_distance = msg.x
        self.current_yaw = msg.y

        rospy.loginfo(f"[{self.robot_name}] Distance: {self.current_distance:.2f}, Yaw: {self.current_yaw:.2f}")
        self.move_to_target()

    def move_to_target(self):
        """
        Determines and sends the appropriate movement command based on distance.
        """
        twist = Twist()
        if self.current_distance < self.target_distance:
            twist.linear.x = 0.2  # Move forward at a constant speed
            twist.angular.z = 0.0  # No turning
            rospy.loginfo(f"[{self.robot_name}] Moving forward: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        else:
            twist.linear.x = 0.0  # Stop moving
            twist.angular.z = 0.0
            rospy.loginfo(f"[{self.robot_name}] Target distance reached. Stopping.")

        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('basic_mover_rafael')
    BasicMoverRafael()
    rospy.spin()
