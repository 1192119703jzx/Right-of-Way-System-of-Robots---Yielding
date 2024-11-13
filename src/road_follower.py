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
yellow_range_lower = np.array([20, 100, 100])
yellow_range_upper = np.array([30, 255, 255])

class RoadFollower:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.PID_FOR_IMAGE = [0.3, 0.000000005, 0.5, 0.05] 
        self.PID_FOR_WALL = [0.8, 0.01, 0.8, 0.05]
        self.linear_speed = 0.075
        self.rate = rospy.Rate(1/self.PID_FOR_IMAGE[3])
        self.ranges = []
        self.has_right = False
        self.has_left = False
        self.avoiding = False
        self.w = 0
        self.x = 0
        self.x2 = 0

    def image_cb(self, msg):
        """Callback to `self.image_sub`."""
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            print(e)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #leftline point
        mask_left = cv2.inRange(hsv_image, red_range_lower, red_range_upper)
        h, w = mask_lect.shape
        slice2 = mask_lect[3*h//4 - 40: 3*h//4 - 20, :] #for find line in front of the robot
        slice3 = mask_lect[1*h//2 : h, :] #find line in the half image
        mu = cv2.moments(slice2)
        contours, _ = cv2.findContours(slice3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if mu["m00"] != 0:
            centroidX = int(mu["m10"] / mu["m00"])
            centroidY = 3*h//4 - 30
            self.has_left = True
        elif contours and self.avoiding == False:
            cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)
            print("in contour")
            centroidX = 0
            centroidY = 0
            min_dist = float('inf')
            for contour in contours:
                mu3 = cv2.moments(contour)
                if mu3["m00"] != 0:
                    cX = int(mu3["m10"] / mu3["m00"])
                    cY = int(mu3["m01"] / mu3["m00"])
                    dist = math.sqrt((cX - w // 2) ** 2 + (cY - h) ** 2)
                    if dist < min_dist:
                        min_dist = dist
                        centroidX = cX
                        centroidY = cY
            if centroidX != 0:
                print("far line")
                self.has_left = True
            else:
                print("fail line")
                self.has_left = False
                centroidX = w // 2
                centroidY = h // 2  
        else:
            centroidX = w // 2
            centroidY = h // 2
            self.has_left = False

        #right line point
        mask_right = cv2.inRange(hsv_image, yellow_range_lower, brown_range_upper)
        h, w = mask_right.shape
        slice4 = mask_right[3*h//4 - 40: 3*h//4 - 20, :]
        mu2 = cv2.moments(slice2)
        if mu2["m00"] != 0:
            centroidX2 = int(mu2["m10"] / mu2["m00"])
            centroidY2 = 3*h//4 - 30
            self.has_right = True
        else:
            centroidX2 = w // 2
            centroidY2 = h // 2
            self.has_right = False

        self.w = w
        self.x = centroidX
        self.x2 = centroidX2
        cv2.circle(cv_image, (centroidX, centroidY), 10, (0, 0, 255), -1)
        cv2.circle(cv_image, (centroidX2, centroidY2), 10, (0, 0, 255), -1)
        cv2.imshow("window", cv_image)
        cv2.waitKey(3)

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


    def follow_line_right(self):
        twist = Twist()
        prev_error = 0
        sum_error = 0
        while not rospy.is_shutdown():
            front = self.cleanMin(self.ranges[0:10] + self.ranges[350:359])
            if self.has_line == True and self.has_wall:
                print("Drive on the right side of the road")
                curr_error = (self.w / 2 - (self.x + (self.x2 - self.x) / 2)) / 100
                sum_error += curr_error * self.PID_FOR_IMAGE[3]
                PID_cal = self.PID_FOR_IMAGE[0] * curr_error #+ self.PID_FOR_IMAGE[2] * (curr_error - prev_error) / self.PID_FOR_IMAGE[3] + self.PID_FOR_IMAGE[1] * sum_error 
                prev_error = curr_error
                twist.angular.z = PID_cal
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            elif front < 0.5:
                self.avoiding_obstacle()
            else:
                rospy.loginfo('no line available, searching for new line')
                twist.angular.z = -0.5
                twist.linear.x = 0
                self.cmd_vel_pub.publish(twist)
            self.rate.sleep()


    def avoiding_obstacle(self):
        twist = Twist()
        prev_error = 0
        sum_error = 0
        self.avoiding = True
        while not rospy.is_shutdown():
            if self.has_line == True:
                self.avoiding = False
                return 
            dist = float('inf')
            front = self.cleanMin(self.ranges[0:10] + self.ranges[350:359])
            left = self.cleanMin(self.ranges[15:60])
            if front < 0.5:
                rospy.loginfo("encouter obstacle, turning")
                twist.angular.z = -1.0
                twist.linear.x = 0
                self.cmd_vel_pub.publish(twist)
            elif left > 0.5:
                rospy.loginfo('turning obstacle corner')
                twist.angular.z = 0.8
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)
            else:
                rospy.loginfo('following obstacle')
                dist = self.cleanMin(self.ranges[45:135])
                curr_error = dist - 0.4
                sum_error += curr_error * self.PID_FOR_WALL[3]
                PID_cal = self.PID_FOR_WALL[0] * curr_error + self.PID_FOR_WALL[2] * (curr_error - prev_error) / self.PID_FOR_WALL[3] + self.PID_FOR_WALL[1] * sum_error
                prev_error = curr_error
                twist.angular.z = PID_cal
                twist.linear.x = self.linear_speed
                self.cmd_vel_pub.publish(twist)

    def run(self):
        """Run the Program."""
        self.follow_line()
        #rospy.spin()
           
if __name__ == '__main__':
    rospy.init_node('road_follower')
    RoadFollower().run()
