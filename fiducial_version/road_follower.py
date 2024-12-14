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
        self.plan = "left"
        self.stage = "moving"
        self.pin_id = None
        self.signal = False

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
            slice101 = mask_left[2*h//3: 2*h//3 + 20, :]
            mu101 = cv2.moments(slice101)
            cv2.imshow("impot", slice101)
            if mu101["m00"] != 0:
                centroidX = int(mu101["m10"] / mu101["m00"])
                centroidY = 2*h//3 + 10
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
            slice102 = mask_right[2*h//3: 2*h//3 + 20, :]
            mu102 = cv2.moments(slice102)
            if mu102["m00"] != 0:
                centroidX2 = int(mu102["m10"] / mu102["m00"])
                centroidY2 = 2*h//3 + 10
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
        #cv2.circle(cv_image, (w // 2, cy_green), 10, (0, 0, 255), -1)
        cv2.imshow("window", cv_image)
        cv2.waitKey(3)

    def follow_road(self):
        twist = Twist()
        prev_error = 0
        sum_error = 0
        print("1")
        while not rospy.is_shutdown():
            print(f"{self.has_left},{self.has_right}")
            if self.pin_id == None:
                pin_frames = []
                while not pin_frames and not rospy.is_shutdown():
                    for line in self.tf_buffer.all_frames_as_string().split('\n'):
                        if 'pin_' in line and 'parent odom' in line:
                            pin_frames.append(line.split()[1])
                    if not pin_frames:
                        rospy.logwarn("No pin frames found, waiting...")
                        rospy.sleep(0.1) 
                pin_id = int(pin_frames[0].split('_')[1])
                self.pin_id = pin_id
                print(self.pin_id)
            try:
                A_tf = self.tf_buffer.lookup_transform("base_link", f"pin_{self.pin_id}", rospy.Time())
                dist = math.sqrt(A_tf.transform.translation.x**2 + A_tf.transform.translation.y**2)
                print(f"distance to fid = {dist}")
                if dist < 0.7:
                    print("got you")
                    self.stage = "cross"
                    #send msg to scheduler node at this line indicate enter waiting stage
                    break
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ):
                pass
            if self.has_left and self.has_right:
                print("Drive on the right side of the road")
                curr_error = (self.w / 2 - (self.x + (self.x2 - self.x) / 2)) / 100
                sum_error += curr_error * self.PID_FOR_IMAGE[3]
                PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                prev_error = curr_error
                twist.angular.z = PID_cal
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            elif not self.has_right:
                curr_error = (self.w / 2 - (self.x + 190)) / 100
                sum_error += curr_error * self.PID_FOR_IMAGE[3]
                PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                prev_error = curr_error
                twist.angular.z = PID_cal
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def face_pin(self):
        try:
            A_tf = self.tf_buffer.lookup_transform("base_link", f"pin_{self.pin_id}", rospy.Time())
            target_yaw = math.atan2(A_tf.transform.translation.y, A_tf.transform.translation.x) 
            twist = Twist()
            init_time = rospy.Time.now()
            turned_angle = 0
            if target_yaw > 0:
                d = 1
            elif target_yaw < 0:
                d = -1
                target_yaw = -target_yaw
            else:
                return
            while turned_angle < target_yaw:
                twist.angular.z = 0.2 * d
                self.cmd_vel_pub.publish(twist)
                turned_angle = 0.2 * (rospy.Time.now() - init_time).to_sec()
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException
            ):
            pass

    def move_to_pin(self):
        twist = Twist()
        while not rospy.is_shutdown():
            try:
                A_tf = self.tf_buffer.lookup_transform("base_link", f"pin_{self.pin_id}", rospy.Time())
                dist = math.sqrt(A_tf.transform.translation.x**2 + A_tf.transform.translation.y**2)
                print(f"I am {dist} to fid")
                if dist > 0.23:
                    target_yaw = math.atan2(A_tf.transform.translation.y, A_tf.transform.translation.x)
                    twist.linear.x = self.linear_speed
                    twist.angular.z = target_yaw
                    self.cmd_vel_pub.publish(twist)
                else:
                    break
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ):
                pass

    def find_line(self):
        twist = Twist()
        while not rospy.is_shutdown():
            if self.has_right and self.has_left:
                twist.linear.x = 0
                twist.angular.z = 0
                self.cmd_vel_pub.publish(twist)
                break
            else:
                twist.linear.x = 0
                twist.angular.z = 0.2
                self.cmd_vel_pub.publish(twist)

    def cross(self):
        twist = Twist()
        prev_error = 0
        sum_error = 0
        print("2")
        while not rospy.is_shutdown():
            print()
            if self.plan == "ahead":
                if self.has_left and self.has_right:
                    self.stage = "moving"
                    # send msg to scheduler node at this line indicate leave stage
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
                    curr_error = (self.w / 2 - (self.x + 190)) / 100
                    sum_error += curr_error * self.PID_FOR_IMAGE[3]
                    PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                    prev_error = curr_error
                    twist.angular.z = PID_cal
                    twist.linear.x = self.linear_speed
                    self.cmd_vel_pub.publish(twist)
                else:
                    self.stage = "moving"
                    # send msg to scheduler node at this line indicate leave stage
                    break
            elif self.plan == "left":
                self.face_pin()
                self.move_to_pin()
                self.find_line()
                self.stage = "moving"
                # send msg to scheduler node at this line indicate leave stage
                break
            self.rate.sleep()

    def follow_continue(self):
        twist = Twist()
        prev_error = 0
        sum_error = 0
        print("3")
        while not rospy.is_shutdown():
            print(f"{self.has_left},{self.has_right}")
            if self.has_left and self.has_right:
                print("Drive on the right side of the road")
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
            else:
                twist.angular.z = 0
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def run(self):
        """Run the Program."""
        self.follow_road()
        rospy.sleep(1)
        self.signal = True
        # Require a call back function and set the self.signal = True in the callback
        #Here wait for self.signal to be True
        if self.signal:
            self.cross()
        self.follow_continue()

           
if __name__ == '__main__':
    rospy.init_node('road_follower')
    RoadFollower().run()
