#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
import numpy as np
import math

red_range_lower = np.array([175, 100, 100])
red_range_upper = np.array([180, 255, 255])
black_range_lower = np.array([0, 0, 0])
black_range_upper = np.array([50, 50, 50])
blue_range_lower = np.array([100, 100, 128])
blue_range_upper = np.array([200, 200, 255])

class RoadFollower:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.PID_FOR_IMAGE = [0.3, 0.000000005, 0.5, 0.05] 
        self.PID_FOR_WALL = [0.8, 0.01, 0.8, 0.05]
        self.linear_speed = 0.075
        self.rate = rospy.Rate(1/self.PID_FOR_IMAGE[3])
        self.ranges = []
        self.has_right = False
        self.has_left = False
        self.w = 0
        self.x = 0
        self.x2 = 0
        self.stop = False

    def image_cb(self, msg):
        """Callback to `self.image_sub`."""
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            print(e)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #leftline point
        mask_left = cv2.inRange(hsv_image, red_range_lower, red_range_upper)
        h, w = mask_left.shape
        slice1 = mask_left[5*h//6: 5*h//6 + 20, :] #for find line in front of the robot
        mu = cv2.moments(slice1)
        if mu["m00"] != 0:
            centroidX = int(mu["m10"] / mu["m00"])
            centroidY = 3*h//4 + 10
            self.has_left = True
        else:
            centroidX = w // 2
            centroidY = h // 2
            self.has_left = False

        #right line point
        mask_right = cv2.inRange(hsv_image, black_range_lower, black_range_upper)
        h, w = mask_right.shape
        slice2 = mask_right[5*h//6: 5*h//6 + 20, :]
        mu2 = cv2.moments(slice2)
        if mu2["m00"] != 0:
            centroidX2 = int(mu2["m10"] / mu2["m00"])
            centroidY2 = 3*h//4 + 10
            self.has_right = True
        else:
            centroidX2 = w // 2
            centroidY2 = h // 2
            self.has_right = False

        self.w = w
        self.x = centroidX
        self.x2 = centroidX2

        #find stop line 
        mask_stop = cv2.inRange(hsv_image, blue_range_lower, blue_range_upper)
        h, w = mask_stop.shape
        slice3 = mask_stop[5*h//6: 5*h//6 + 20, :]
        mu4 = cv2.moments(slice3)
        #cv2.imshow("window", cv_image[5*h//6: 5*h//6 + 20, :])
        if mu4['m00'] != 0:
            cy_blue = int(mu4['m01'] / mu4['m00'])
            print("yes")
        else:
            cy_blue = 0
        dist = h - cy_blue
        if dist < 0.5:
            self.stop = True

        cv2.circle(cv_image, (centroidX, centroidY), 10, (0, 0, 255), -1)
        cv2.circle(cv_image, (centroidX2, centroidY2), 10, (0, 0, 0), -1)
        cv2.circle(cv_image, (w // 2, cy_blue), 10, (0, 0, 255), -1)
        cv2.imshow("window", cv_image)
        cv2.waitKey(3)


    def follow_road(self):
        twist = Twist()
        prev_error = 0
        sum_error = 0
        while not rospy.is_shutdown():
            if self.stop:
                break
            elif self.has_left and self.has_right:
                print("Drive on the right side of the road")
                curr_error = (self.w / 2 - (self.x + (self.x2 - self.x) / 2)) / 100
                print((self.x2 - self.x) / 2)
                sum_error += curr_error * self.PID_FOR_IMAGE[3]
                PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                prev_error = curr_error
                twist.angular.z = PID_cal
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            elif self.has_left:
                print("Drive on the right side of the road, black line invisible")
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

    def run(self):
        """Run the Program."""
        self.follow_road()
        #rospy.spin()
           
if __name__ == '__main__':
    rospy.init_node('road_follower')
    RoadFollower().run()
