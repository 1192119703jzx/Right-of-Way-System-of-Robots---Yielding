#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String

class BasicMover:
    def __init__(self):
        # Retrieve namespace and topics
        #self.robot_namespace = rospy.get_param('~robot_namespace', '')
        odom_topic = rospy.get_param('~odom_topic', f"/my_odom")
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic', f"/cmd_vel")

        # Log the topics being used
        rospy.loginfo(f"BasicMover initialized for namespace: roba")
        rospy.loginfo(f"Subscribing to: {odom_topic}")
        rospy.loginfo(f"Publishing to: {cmd_vel_topic}")

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber(odom_topic, Point, self.odom_cb)
        self.state_pub = rospy.Publisher("/robot_states", String, queue_size=10)
        self.command_sub = rospy.Subscriber("/scheduler_commands", String, self.command_cb)

        # Movement control variables
        self.target_distance = 0.05  # First target distance
        self.second_target_distance = 0.20  # Second target distance in Passing stage
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

        rospy.loginfo(f"[roba] State: {self.state}, Distance: {self.current_distance:.2f}, Yaw: {self.current_yaw:.2f}")
        self.handle_state()

    def handle_state(self):
        """
        Manage robot behavior and state transitions.
        """
        twist = Twist()

        if self.state == "Entering":
            if self.current_distance < self.target_distance:
                twist.linear.x = 0.1  # Move forward
                rospy.loginfo(f"[roba] Entering: Moving forward.")
            else:
                twist.linear.x = 0.0  # Stop moving
                self.state = "Waiting"
                rospy.loginfo(f"[roba] Transition to Waiting.")

        elif self.state == "Waiting":
            # Simulate waiting behavior
            rospy.loginfo(f"[roba] Waiting at target.")
            rospy.sleep(1)  # Waiting stage handled by scheduler

        elif self.state == "Passing":
            if self.current_distance < self.second_target_distance:
                print("[roba] Passing distance:", self.current_distance)
                print("[roba] target distance:", self.second_target_distance)
                twist.linear.x = 0.2  # Move faster during Passing
                rospy.loginfo(f"[roba] Passing: Moving to second target.")
            else:
                twist.linear.x = 0.0  # Stop moving
                self.state = "Left"
                rospy.loginfo(f"[roba] Transition to Left.")

        self.cmd_vel_pub.publish(twist)
        self.publish_state()

    def command_cb(self, msg):
        """
        Handle commands from the scheduler.
        """
        print("Has we ARRRRRRRRRRR")
        command = msg.data.split(", ")
        if len(command) == 2 and self.state != "Left" and self.state != "Passing":
            #print("Yes command")
            robot_name, action = command
            #print("rob name:",robot_name)
            #print("name space:", self.robot_namespace)
            if robot_name == "roba" and action == "Passing":
                rospy.loginfo(f"[roba] Received command to enter Passing stage.")
                self.state = "Passing"
                self.second_target_distance = self.current_distance + 0.2
                rospy.loginfo(f"[roba] New second target distance: {self.second_target_distance:.2f}")


    def publish_state(self):
        """
        Publish the robot's current state to the scheduler.
        """
        state_msg = f"roba, {self.state}"
        rospy.loginfo(f"[roba] Publishing state: {state_msg}")
        self.state_pub.publish(state_msg)

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    BasicMover()
    rospy.spin()
