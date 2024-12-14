#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from fiducial_msgs.msg import FiducialTransformArray

class FiducialDist:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pin_dict = {}
        self.fid = ""

    def add_fiducial(self, fid_id):
        """
        Add a new fiducial to the pin_dict.
        """
        tfs = TransformStamped()
        tfs.header.frame_id = 'rafael/odom'
        tfs.child_frame_id = f'pin_{fid_id}'
        self.pin_dict[fid_id] = {'tfs': tfs}

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Get the list of fiducial frames from the tf tree
            if not self.fid:
                fiducial_frames = []
                for line in self.tf_buffer.all_frames_as_string().split('\n'):
                    if 'fiducial_' in line and 'parent rafael/raspicam' in line:
                        fiducial_frames.append(line.split()[1])
                if fiducial_frames:
                    self.fid = fiducial_frames[0]
            else:
                fid_id = int(self.fid.split('_')[1])
                try:
                    odom_to_fid_tf = self.tf_buffer.lookup_transform(
                            'rafael/odom',  # Target frame
                            self.fid,  # Source frame
                            rospy.Time()
                        ).transform
                    
                    self.add_fiducial(fid_id)
                    pin = self.pin_dict[fid_id]
                    pin['tfs'].transform.rotation = odom_to_fid_tf.rotation
                    pin['tfs'].transform.translation = odom_to_fid_tf.translation

                    # Ensure the transform is correctly updated
                    pin['tfs'].header.stamp = rospy.Time.now()
                    self.tf_broadcaster.sendTransform(pin['tfs'])
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException
                    ):
                    pass
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fiducial_dist')
    mapper = FiducialDist()
    mapper.run()