#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from time import time
from collections import deque

class Scheduler:
    def __init__(self):
        rospy.init_node('scheduler_node', anonymous=True)

        # Queue to store received messages with time of arrival
        self.message_queue = deque()

        # Publisher & Subscriber
        self.response_publisher = rospy.Publisher('response_info', String, queue_size=10)
        self.subscriber = rospy.Subscriber('robot_info', String, self.callback)

        self.rate = rospy.Rate(1)

    def callback(self, data):
        """Callback function to process incoming messages."""
        arrival_time = time()  # Record the time of arrival
        message = data.data

        # Log the received message
        rospy.loginfo(f"Received: {message} at {arrival_time}")

        # Add the message and time to the queue
        self.message_queue.append({"message": message, "arrival_time": arrival_time})

    def schedule(self):
        """Process the queue and handle messages."""
        while not rospy.is_shutdown():
            if self.message_queue:
                # Sort the queue by arrival time (FCFS scheduling)
                self.message_queue = deque(sorted(self.message_queue, key=lambda x: x["arrival_time"]))

                # Get the next message
                task = self.message_queue.popleft()

                # Log and respond to the message
                rospy.loginfo(f"Processing: {task['message']} received at {task['arrival_time']}")
                response = f"Processed: {task['message']}"
                self.response_publisher.publish(response)
            else:
                rospy.loginfo("No messages in the queue.")

            # Wait for the next iteration
            self.rate.sleep()

if __name__ == "__main__":
    try:
        scheduler = Scheduler()
        scheduler.schedule()
    except rospy.ROSInterruptException:
        rospy.loginfo("Scheduler terminated.")
