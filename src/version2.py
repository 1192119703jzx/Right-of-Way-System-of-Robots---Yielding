#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy
import tf2_ros
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import numpy as np
import math
import json

red_range_lower = np.array([0, 100, 100])
red_range_upper = np.array([10, 255, 255])
black_range_lower = np.array([0, 0, 0])
black_range_upper = np.array([180, 255, 50])


class RoadFollower:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        #scheduler communication
        self.state_pub = rospy.Publisher("/robot_states", String, queue_size=10)
        self.command_sub = rospy.Subscriber("/scheduler_commands", String, self.command_cb)
        self.state = "Entering"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.PID_FOR_IMAGE = [0.3, 0.000000005, 0.5, 0.05] 
        self.PID_FOR_WALL = [0.8, 0.01, 0.8, 0.05]
        self.linear_speed = 0.075
        self.rate = rospy.Rate(1/self.PID_FOR_IMAGE[3])
        self.has_right = False
        self.has_right2 = False
        self.has_left = False
        self.has_left2 = False
        self.w = 0
        self.x = 0
        self.x2 = 0
        self.plan = "ahead"
        self.stage = "moving"
        self.pin_id = None
        self.ranges = []
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

    def cleanMin(self, ranges):
        new_ranges = list(ranges)
        new_ranges.sort()
        new_ranges = [num for num in new_ranges if 0.07 <= num and not math.isnan(num)]
        if len(new_ranges) >= 10:
            sample = 10
        else:
            sample = len(new_ranges)
        if sample == 0:
            return float('inf')
        else:
            average_min= sum(new_ranges[0:sample]) / sample
            return average_min

    def scan_cb(self,msg):
        self.ranges = msg.ranges

    def image_cb(self, msg):
        """Callback to `self.image_sub`."""
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            print(e)

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #leftline point
        mask_left = cv2.inRange(hsv_image, red_range_lower, red_range_upper)
        cv2.imshow("Mask Left", mask_left)
        h, w = mask_left.shape
        slice1 = mask_left[5*h//6: 5*h//6 + 20, :] #for find line in front of the robot
        mu = cv2.moments(slice1)
        if self.stage == "cross" and self.plan == "ahead":
            slice101 = mask_left[1*h//2: 1*h//2 + 20, :]
            mu101 = cv2.moments(slice101)
            if mu101["m00"] != 0:
                centroidX = int(mu101["m10"] / mu101["m00"])
                centroidY = 1*h//2 + 10
                self.has_left2 = True
            else:
                centroidX = w // 2
                centroidY = h // 2
                self.has_left2 = False
            if mu["m00"] != 0:
                self.has_left = True
        else:
            if mu["m00"] != 0:
                centroidX = int(mu["m10"] / mu["m00"])
                centroidY = 5*h//6 + 10
                self.has_left = True
            else:
                centroidX = w // 2
                centroidY = h // 2
                self.has_left = False

        #right line point
        mask_right = cv2.inRange(hsv_image, black_range_lower, black_range_upper)
        cv2.imshow("Mask Right", mask_right)
        h, w = mask_right.shape
        slice2 = mask_right[5*h//6: 5*h//6 + 20, :]
        mu2 = cv2.moments(slice2)
        if self.stage == "cross" and self.plan == "ahead":
            slice102 = mask_right[1*h//2: 1*h//2 + 20, :]
            mu102 = cv2.moments(slice102)
            if mu102["m00"] != 0:
                centroidX2 = int(mu102["m10"] / mu102["m00"])
                centroidY2 = 1*h//2 + 10
                self.has_right2 = True
            else:
                centroidX2 = w // 2
                centroidY2 = h // 2
                self.has_right2 = False
            if mu2["m00"] != 0:
                self.has_right = True
        else:
            if mu2["m00"] != 0:
                centroidX2 = int(mu2["m10"] / mu2["m00"])
                centroidY2 = 5*h//6 + 10
                self.has_right = True
            else:
                self.has_right = False
                centroidX2 = w // 2
                centroidY2 = h // 2

        self.w = w
        self.x = centroidX
        self.x2 = centroidX2

        cv2.circle(cv_image, (centroidX, centroidY), 10, (0, 0, 255), -1)
        cv2.circle(cv_image, (centroidX2, centroidY2), 10, (0, 0, 0), -1)
        cv2.imshow("window", cv_image)
        cv2.waitKey(3)

    def follow_road(self):
        twist = Twist()
        prev_error = 0
        sum_error = 0
        print("1")
        while not rospy.is_shutdown():
            print(f"roba {self.has_left},{self.has_right}, moving1")
            left = self.cleanMin(self.ranges[300:345])
            print(left)
            if left == float('inf'):
                continue
            elif left > 0.35:
                print("got you")
                self.stage = "cross"
                self.state = "Waiting"
                self.publish_state()
                break
            if self.has_left and self.has_right:
                curr_error = (self.w / 2 - (self.x + (self.x2 - self.x) / 2)) / 100
                sum_error += curr_error * self.PID_FOR_IMAGE[3]
                PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                prev_error = curr_error
                twist.angular.z = PID_cal
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            elif not self.has_right:
                twist.angular.z = 0
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def cross(self):
        twist = Twist()
        prev_error = 0
        sum_error = 0
        print("2")
        while not rospy.is_shutdown():
            print()

            #Passing check
            if self.state == "Passing":
                print("YES YES")

            if self.plan == "ahead":
                print("going ahead")
                if self.has_left and self.has_right:
                    self.stage = "moving"
                    self.state = "Left"
                    self.publish_state()
                    break
                elif self.has_left2:
                    curr_error = (self.w / 2 - (self.x + 95)) / 100
                    sum_error += curr_error * self.PID_FOR_IMAGE[3]
                    PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                    prev_error = curr_error
                    twist.angular.z = PID_cal
                    twist.linear.x = self.linear_speed
                    self.cmd_vel_pub.publish(twist)
            elif self.plan == "right":
                if not self.has_right:
                    curr_error = (self.w / 2 - (self.x + 160)) / 100
                    sum_error += curr_error * self.PID_FOR_IMAGE[3]
                    PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                    prev_error = curr_error
                    twist.angular.z = PID_cal
                    twist.linear.x = 0.06
                    self.cmd_vel_pub.publish(twist)
                else:
                    self.stage = "moving"
                    self.state = "Left"
                    self.publish_state()
                    break
            elif self.plan == "left":
                if self.has_right and self.has_left:
                    self.stage = "moving"
                    self.state = "Left"
                    self.publish_state()
                else:
                    twist.angular.z = 0.1875
                    twist.linear.x = self.linear_speed
                    self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

    def follow_continue(self):
        twist = Twist()
        prev_error = 0
        sum_error = 0
        print("3")
        while not rospy.is_shutdown():
            print(f"roba {self.has_left},{self.has_right}, leaving")
            if self.has_left and self.has_right:
                curr_error = (self.w / 2 - (self.x + (self.x2 - self.x) / 2)) / 100
                sum_error += curr_error * self.PID_FOR_IMAGE[3]
                PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                prev_error = curr_error
                twist.angular.z = PID_cal
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            elif self.has_left:
                    curr_error = (self.w / 2 - (self.x + 190)) / 100
                    sum_error += curr_error * self.PID_FOR_IMAGE[3]
                    PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                    prev_error = curr_error
                    twist.angular.z = PID_cal
                    twist.linear.x = self.linear_speed
                    self.cmd_vel_pub.publish(twist)
            elif self.has_right:
                    curr_error = (self.w / 2 - (self.x2 - 95)) / 100
                    sum_error += curr_error * self.PID_FOR_IMAGE[3]
                    PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                    prev_error = curr_error
                    twist.angular.z = PID_cal
                    twist.linear.x = self.linear_speed
                    self.cmd_vel_pub.publish(twist)
            else:
                twist.angular.z = 0
                twist.linear.x = 0
                self.cmd_vel_pub.publish(twist)
                break

            self.rate.sleep()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)
    
    def command_cb(self, msg):
        """
        Handle commands from the scheduler.
        """
        command = msg.data.split(", ")
        if len(command) == 2 and self.state != "Left" and self.state != "Passing":
            robot_name, action = command
            if robot_name == "roba" and action == "Passing":
                rospy.loginfo(f"[roba] Received command to enter Passing stage.")
                self.state = "Passing"

    def publish_state(self):
        """
        Publish the robot's current state to the scheduler.
        """
        state_msg = f"roba, {self.state}"
        rospy.loginfo(f"[roba] Publishing state: {state_msg}")
        self.state_pub.publish(state_msg)

    def run(self):
        """Run the Program."""
        self.follow_road()
        while not rospy.is_shutdown():
            if self.state == "Passing":
                print("YES YES")
            else: 
                print("RobA, You shall not pass")

            if self.state == "Passing":
                self.cross()
                break

            self.rate.sleep()
        self.follow_continue()
        

           
if __name__ == '__main__':
    rospy.init_node('road_follower_rafael')
    RoadFollower().run()
