#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point

class BasicSetup:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.init_states()

        self.linear_speed = 0.2  # Speed to move forward
        self.turning_speed = 0.1
        self.angular_speed = 0.4  # Speed to rotate
        self.turning_augular = 0.5


        # Initialize Twist message
        self.cmd = Twist()
        self.cmd.linear.x = self.linear_speed
        self.cmd.angular.z = 0.0

        #image size
        self.width = 0
        self.height = 0

        # Contour center 
        self.cx = None
        self.cy = None

        #Odemetry
        self.cur_yaw = None
        self.start_yaw = None
        self.target_dist = 0
        self.distance_moved = 0 

        # Current LIDAR data
        self.regions = {
            'front': float('inf'),
            'left': float('inf'),
            'right': float('inf')
        }

        self.check = True

        


    def init_states(self):
        """Initialize the states of the robot."""
        self.states = {
            'following_line': True,
            'avoiding_obstacle': False,
        }

    def my_odom_cb(self, msg):
        """Callback to update distance moved and yaw."""
        self.distance_moved = msg.x  
        self.cur_yaw = msg.y
        
        # Add debug log for cur_yaw
        rospy.loginfo(f"Current Yaw Updated: {self.cur_yaw}")
        
        if self.target_dist is not None and abs(self.distance_moved) >= abs(self.target_dist):
            rospy.loginfo("Target distance reached. Stopping.")
            self.cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(self.cmd)
            self.target_dist = None

    def image_cb(self, msg):
        """Callback to `self.image_sub`."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.height, self.width, _ = cv_image.shape
            
            # Convert to HSV color space
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define HSV range for red

            lower_red = np.array([170, 100, 100])
            upper_red = np.array([180, 255, 255])

            mask = cv2.inRange(hsv_image, lower_red, upper_red)

            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    self.cx = int(M["m10"] / M["m00"])  # x-coordinate
                    self.cy = int(M["m01"] / M["m00"])  # y-coordinate
                    cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 3)
                    cv2.circle(cv_image, (self.cx, self.cy), 5, (255, 0, 0), -1)
            else:
                self.cx = None  

            # Display image
            cv2.imshow("window", cv_image)
            cv2.waitKey(1)
        
        except Exception as e:
            rospy.logerr("Failed to process image for yellow line detection: %s", str(e))


    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""

        if not hasattr(self, 'angle_min'):  
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            #rospy.loginfo(f"LIDAR Parameters: angle_min={self.angle_min}, angle_max={msg.angle_max}, angle_increment={self.angle_increment}")

        # Calculate indices for front, left, and right regions
        angle_front_min = -15 * (math.pi / 180)
        angle_front_max = 15 * (math.pi / 180)
        index_front_min = int((angle_front_min - self.angle_min) / self.angle_increment)
        index_front_max = int((angle_front_max - self.angle_min) / self.angle_increment)

        angle_left_min = 45 * (math.pi / 180)
        angle_left_max = 90 * (math.pi / 180)
        index_left_min = int((angle_left_min - self.angle_min) / self.angle_increment)
        index_left_max = int((angle_left_max - self.angle_min) / self.angle_increment)

        angle_right_min = -90 * (math.pi / 180)
        angle_right_max = -45 * (math.pi / 180)
        index_right_min = int((angle_right_min - self.angle_min) / self.angle_increment)
        index_right_max = int((angle_right_max - self.angle_min) / self.angle_increment)

        # Apply median filter to the relevant indices
        front_ranges = [msg.ranges[i] for i in range(index_front_min, index_front_max + 1) if msg.ranges[i] < float('inf')]
        left_ranges = [msg.ranges[i] for i in range(index_left_min, index_left_max + 1) if msg.ranges[i] < float('inf')]
        right_ranges = [msg.ranges[i] for i in range(index_right_min, index_right_max + 1) if msg.ranges[i] < float('inf')]

        self.regions['front'] = np.median(front_ranges) if front_ranges else float('inf')
        self.regions['left'] = np.median(left_ranges) if left_ranges else float('inf')
        self.regions['right'] = np.median(right_ranges) if right_ranges else float('inf')
  
           
