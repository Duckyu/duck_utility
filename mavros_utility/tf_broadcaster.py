#!/usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import PoseStamped
# from tf2_msgs.msg import TFMessage
import tf
import tf_conversions
import tf2_ros
import sys
import signal

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

class caster():
    def __init__(self):
        rospy.init_node('cvtPose2TF', anonymous=True)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.base_cb)
        self.rate = rospy.Rate(30)
        self.base_check=0
        self.br = tf.TransformBroadcaster()
        self.br = tf.TransformBroadcaster()

    def base_cb(self, msg):
        self.base_check=1
        self.header = msg.header.stamp
        self.br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),\
(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),\
self.header,"base_link","map")
        self.br.sendTransform((0, 0, 0.05), (0,0,-0.7071067,0.7071067), self.header,"up_camera_link","base_link")
        self.br.sendTransform((0.1, 0, 0), (-0.5,-0.5,-0.5,0.5), self.header,"front_camera_link","base_link")
        return

if __name__ == '__main__':
    cas = caster()
    while 1:
        try:
            cas.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
