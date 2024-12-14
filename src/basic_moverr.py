#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String

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

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber(odom_topic, Point, self.odom_cb)
        self.state_pub = rospy.Publisher("/robot_states", String, queue_size=10)
        self.command_sub = rospy.Subscriber("/scheduler_commands", String, self.command_cb)

        # Movement control variables
        self.target_distance = 0.18  # First target distance
        self.second_target_distance = 0.2  # Second target distance in Passing stage
        self.current_distance = 0.0
        self.current_yaw = 0.0

        # Robot state management
        self.state = "Entering"

    def odom_cb(self, msg):
        """
        Callback for processed odometry data.
        Updates the distance traveled and yaw, and manages the state transitions.
        """
        self.current_distance = msg.x
        self.current_yaw = msg.y

        rospy.loginfo(f"[{self.robot_name}] State: {self.state}, Distance: {self.current_distance:.2f}, Yaw: {self.current_yaw:.2f}")
        self.handle_state()

    def handle_state(self):
        """
        Manage robot behavior and state transitions.
        """
        twist = Twist()

        if self.state == "Entering":
            if self.current_distance < self.target_distance:
                twist.linear.x = 0.1  # Move forward
                rospy.loginfo(f"[{self.robot_name}] Entering: Moving forward.")
            else:
                twist.linear.x = 0.0  # Stop moving
                self.state = "Waiting"
                rospy.loginfo(f"[{self.robot_name}] Transition to Waiting.")

        elif self.state == "Waiting":
            # Publish current state
            self.publish_state()
            rospy.loginfo(f"[{self.robot_name}] Waiting at target.")

        elif self.state == "Passing":
            if self.current_distance < self.second_target_distance:
                print("[ra] Passing distance:", self.current_distance)
                print("[ra] target distance:", self.second_target_distance)
                twist.linear.x = 0.2  # Move faster during Passing
                rospy.loginfo(f"[{self.robot_name}] Passing: Moving to second target.")
            else:
                twist.linear.x = 0.0  # Stop moving
                self.state = "Left"
                rospy.loginfo(f"[{self.robot_name}] Transition to Left.")

        elif self.state == "Left":
            twist.linear.x = 0.0  # Stop completely
            rospy.loginfo(f"[{self.robot_name}] Left: Robot has exited.")

        self.cmd_vel_pub.publish(twist)
        self.publish_state()

    def command_cb(self, msg):
        """
        Handle commands from the scheduler.
        """
        command = msg.data.split(", ")
        if len(command) == 2 and self.state != "Left" and self.state != "Passing":
            robot_name, action = command
            if robot_name == self.robot_namespace and action == "Passing":
                rospy.loginfo(f"[{self.robot_name}] Received command to enter Passing stage.")
                self.state = "Passing"
                self.second_target_distance = self.current_distance + 0.2
                rospy.loginfo(f"[roba] New second target distance: {self.second_target_distance:.2f}")

    def publish_state(self):
        """
        Publish the robot's current state to the scheduler.
        """
        state_msg = f"{self.robot_name}, {self.state}"
        rospy.loginfo(f"[{self.robot_name}] Publishing state: {state_msg}")
        self.state_pub.publish(state_msg)

if __name__ == '__main__':
    rospy.init_node('basic_mover_rafael')
    BasicMoverRafael()
    rospy.spin()






