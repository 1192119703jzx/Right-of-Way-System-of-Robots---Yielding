#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Pose 
from tf.transformations import euler_from_quaternion

#Description video here
#https://drive.google.com/drive/folders/1iP5pB50MMlHbeLn-_x5xMzNuaXOtwDUZ?usp=sharing

class MyOdom:
    def __init__(self):
        self.robot_namespace = rospy.get_param('~robot_namespace', '')
        self.robot_name = self.robot_namespace if self.robot_namespace else 'roba'
        odom_topic = rospy.get_param('~odom_topic', '/odom')
        
        # Log the robot name
        rospy.loginfo(f"MyOdom initialized for robot: {self.robot_name}")
        
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher(f"{self.robot_namespace}/my_odom", Point, queue_size=1)
        



        print("Here")

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
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """

        if self.old_pose is not None:

            print("Updating")

            self.dist += self.calculate_distance(cur_pose.position, self.old_pose.position)
            self.update_position(cur_pose.position)
        
        else:
            print("Empty Position")
            self.old_pose = Pose()
            self.update_position(cur_pose.position)





    def update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """

        self.yaw = self.get_rotation(cur_orientation)




    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        point_msg = Point()
        point_msg.x = self.dist  # Store distance
        point_msg.y = self.yaw   # Store yaw 
        point_msg.z = 0.0        

        # Publish the point message on the 'my_odom' topic
        rospy.loginfo(f"[{self.robot_name}]: Publishing Distance: {self.dist}, Yaw: {self.yaw}")
        self.my_odom_pub.publish(point_msg)


        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.
        # raise NotImplementedError


    #Even more helper functions 

    def calculate_distance(self, new_pos, old_pos):
        """Calculate the distance between two Points (positions)."""
        x2 = new_pos.x
        x1 = old_pos.x
        y2 = new_pos.y
        y1 = old_pos.y
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist
    

    #Get correct yaw
    def get_rotation (self, cur_orient):
        
        orientation_q = cur_orient
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw


    #update the robot the the newest position
    def update_position(self, new_position):
        """Update the current position of the robot."""
        # Update the old position with the new position values
        self.old_pose.position.x = new_position.x
        self.old_pose.position.y = new_position.y
        self.old_pose.position.z = new_position.z

    



        
if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()