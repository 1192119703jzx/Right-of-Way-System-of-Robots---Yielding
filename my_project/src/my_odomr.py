#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose
from tf.transformations import euler_from_quaternion

class OdomRafael:
    def __init__(self):
        # Namespace specific to Rafael
        self.robot_namespace = 'rafael'
        self.robot_name = 'rafael'

        # Rafael's odometry topic
        odom_topic = f"/{self.robot_namespace}/odom"
        processed_odom_topic = f"/{self.robot_namespace}/my_odom"

        # Log the topics being used
        rospy.loginfo(f"OdomRafael initialized for namespace: {self.robot_namespace}")
        rospy.loginfo(f"Subscribing to: {odom_topic}")
        rospy.loginfo(f"Publishing to: {processed_odom_topic}")

        # Publisher and Subscriber
        self.my_odom_pub = rospy.Publisher(processed_odom_topic, Point, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_cb)

        # Initialize position and other attributes
        self.old_pose = None
        self.dist = 0.0
        self.yaw = 0.0

    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose)
        self.update_yaw(cur_pose.orientation)
        self.publish_data()

    def update_dist(self, cur_pose):
        """Update `self.dist` to the distance between `self.old_pose` and `cur_pose`."""
        if self.old_pose is not None:
            rospy.loginfo(f"[{self.robot_name}] Updating distance.")
            self.dist += self.calculate_distance(cur_pose.position, self.old_pose.position)
            self.update_position(cur_pose.position)
        else:
            rospy.loginfo(f"[{self.robot_name}] Initializing position.")
            self.old_pose = Pose()
            self.update_position(cur_pose.position)

    def update_yaw(self, cur_orientation):
        """Update `self.yaw` to the current heading of the robot."""
        self.yaw = self.get_rotation(cur_orientation)

    def publish_data(self):
        """Publish `self.dist` and `self.yaw` on the `my_odom` topic."""
        point_msg = Point()
        point_msg.x = self.dist  # Store distance
        point_msg.y = self.yaw   # Store yaw
        point_msg.z = 0.0        

        rospy.loginfo(f"[{self.robot_name}]: Publishing Distance: {self.dist}, Yaw: {self.yaw}")
        self.my_odom_pub.publish(point_msg)

    def calculate_distance(self, new_pos, old_pos):
        """Calculate the distance between two Points (positions)."""
        x2 = new_pos.x
        x1 = old_pos.x
        y2 = new_pos.y
        y1 = old_pos.y
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist

    def get_rotation(self, cur_orient):
        """Get the yaw (rotation around z-axis) from the quaternion."""
        orientation_q = cur_orient
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def update_position(self, new_position):
        """Update the current position of the robot."""
        if not self.old_pose:
            self.old_pose = Pose()
        self.old_pose.position.x = new_position.x
        self.old_pose.position.y = new_position.y
        self.old_pose.position.z = new_position.z

if __name__ == '__main__':
    rospy.init_node('odom_rafael')
    OdomRafael()
    rospy.spin()
