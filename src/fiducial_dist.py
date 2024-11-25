#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

class Fiducial_dist:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pin_dict = {}

    def add_fiducial(self, fid_id):
        """
        Add a new fiducial to the pin_dict.
        """
        if fid_id not in self.pin_dict:
            tfs = TransformStamped()
            tfs.header.frame_id = 'odom'
            tfs.child_frame_id = f'pin_{fid_id}'
            self.pin_dict[fid_id] = {'tfs': tfs, 'mapped': False}

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Get the list of fiducial frames from the tf tree
            fiducial_frames = [f for f in self.tf_buffer._frame_cache.keys() if f.startswith('fiducial_')]

            for fiducial_frame in fiducial_frames:
                fid_id = int(fiducial_frame.split('_')[1])
                odom_to_fid_tf = self.tf_buffer.lookup_transform(
                            'odom',
                            fiducial_frame,
                            rospy.Time()).transform

                # Get the orientation of the fiducial
                fid_quaternion = odom_to_fid_tf.rotation
                fid_euler = euler_from_quaternion([
                    fid_quaternion.x,
                    fid_quaternion.y,
                    fid_quaternion.z,
                    fid_quaternion.w
                ])
                fid_yaw = fid_euler[2]

                # Check if the fiducial is facing the robot (within a certain yaw range)
                if -0.5 < fid_yaw < 0.5:  # Adjust the range as needed
                    self.add_fiducial(fid_id)
                    pin['tfs'].transform.translation = odom_to_fid_tf.translation
                    q = quaternion_from_euler(0.0, 0.0, 0.0)
                    (pin['tfs'].transform.rotation.x,
                    pin['tfs'].transform.rotation.y,
                    pin['tfs'].transform.rotation.z,
                    pin['tfs'].transform.rotation.w) = q


            pin['tfs'].header.stamp = rospy.Time.now()
            self.tf_broadcaster.sendTransform(pin['tfs'])
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fiducial_dist')
    mapper = Fiducial_dist()
    mapper.run()
