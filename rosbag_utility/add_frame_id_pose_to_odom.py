import rosbag
import argparse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

parser = argparse.ArgumentParser()
parser.add_argument("-b", "--bag_name", dest="bag", action="store")
parser.add_argument("-o", "--output", dest="output", action="store")
args = parser.parse_args()

with rosbag.Bag(str(args.output), 'w') as outbag:
    for topic, msg, t in rosbag.Bag(str(args.bag)).read_messages():
        # print(msg.__class__.__name__)
        msg.header.frame_id = 'map'
        if msg.__class__.__name__ == "_geometry_msgs__PoseStamped":
            conv = Odometry()
            conv.header = msg.header
            conv.pose.pose = msg.pose
            outbag.write(topic, conv, msg.header.stamp if msg._has_header else t)

        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)