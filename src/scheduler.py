#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class RobotScheduler:
    def __init__(self):
        # Topics for robot states and commands
        self.state_topic = "/robot_states"
        self.command_topic = "/scheduler_commands"
        print("SChe845e41d5151514541s")
        # Subscribers and Publishers
        self.state_sub = rospy.Subscriber(self.state_topic, String, self.state_cb)
        self.command_pub = rospy.Publisher(self.command_topic, String, queue_size=10)
        
        # List to track robots in the scheduler
        self.scheduler_robots = []
        self.is_pass = False
        rospy.loginfo("Robot Scheduler initialized.")

    def state_cb(self, msg):
        
        """
        Callback for robot state updates.
        Updates the list of robots in the scheduler.
        """

        try:
            data = msg.data.split(", ")
            if len(data) == 2:
                robot_name, state = data

                # Handle empty robot_name
                if not robot_name.strip():
                    #rospy.logwarn(f"Received empty robot_name. Renaming to 'roba'.")
                    robot_name = "roba"

                # Update scheduler list
                if state == "Waiting" and robot_name not in self.scheduler_robots:
                    if len(self.scheduler_robots) < 2:
                        self.scheduler_robots.append(robot_name)
                        rospy.loginfo(f"Added {robot_name} to scheduler.")
                elif state == "Left" and robot_name in self.scheduler_robots:
                    self.scheduler_robots.remove(robot_name)
                    rospy.loginfo(f"Removed {robot_name} from scheduler.")

                # Print the robots in the scheduler
                rospy.loginfo(f"Robots in scheduler: {', '.join(self.scheduler_robots)}")

                # Schedule robots if conditions are met
                self.schedule_robots()

            else:
                rospy.logwarn(f"Received invalid state message: {msg.data}")
        except Exception as e:
            rospy.logerr(f"Error processing state message: {e}")

    def schedule_robots(self):
        """
        Schedules robots if conditions are met.
        The first robot is transitioned to "Passing".
        """
        if len(self.scheduler_robots) == 1 and self.is_pass is False:
            self.is_pass = True
            # Change the state of the first robot in the scheduler to "Passing"
            next_robot = self.scheduler_robots[0]
            rospy.loginfo(f"Scheduling robot {next_robot} to Passing stage.")
            self.command_pub.publish(f"{next_robot}, Passing")
        elif self.is_pass is True:
            next_robot = self.scheduler_robots[0]
            rospy.loginfo(f"Scheduling robot {next_robot} to Passing stage.")
            self.command_pub.publish(f"{next_robot}, Passing")
        elif len(self.scheduler_robots) >= 2:
            self.is_pass = False

if __name__ == '__main__':
    rospy.init_node('robot_scheduler')
    scheduler = RobotScheduler()
    rospy.spin()
