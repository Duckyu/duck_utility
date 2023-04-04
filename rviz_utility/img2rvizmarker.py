#!/usr/bin/env python
from PIL import Image
import rospy
import argparse
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
parser = argparse.ArgumentParser()
parser.add_argument("-f", "--file", dest="file", action="store")
parser.add_argument("-s", "--sacle", type=float, dest="scale", action="store")
parser.add_argument("-x", "--offset_x", type=float, dest="offset_x", action="store", default=0.0)
parser.add_argument("-y", "--offset_y", type=float, dest="offset_y", action="store", default=0.0)
args = parser.parse_args()

def pub_convert():
    rospy.init_node('map', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Open the image file
    image = Image.open(str(args.file))
    print(image.size)
    tempMarker = Marker()
    tempMarker.header.frame_id = "map"
    tempMarker.id = 0
    tempMarker.ns = "map"
    tempMarker.type = Marker.CUBE_LIST
    tempMarker.action = Marker.ADD
    tempMarker.pose.position.x = 0.0
    tempMarker.pose.position.y = 0.0
    tempMarker.pose.position.z = 0.0
    tempMarker.pose.orientation.x = 0.0
    tempMarker.pose.orientation.y = 0.0
    tempMarker.pose.orientation.z = 0.0
    tempMarker.pose.orientation.w = 1.0
    tempMarker.scale.x = 1.0 / args.scale
    tempMarker.scale.y = 1.0 / args.scale
    tempMarker.scale.z = 1.0 / args.scale
    tempMarker.color.a = 1.0
    tempMarker.color.r = 1.0
    tempMarker.color.g = 1.0
    tempMarker.color.b = 1.0
    tempMarker.lifetime = rospy.Duration()
    pub_marker = rospy.Publisher('map', Marker, queue_size=10)
       
    # Convert the image to grayscale
    gray_image = image.convert('L')
    img_arr = np.array(gray_image)
    for i in range(img_arr.shape[0]):
        for j in range(img_arr.shape[1]):
            if img_arr[i,j] < 10:
                print(i,j)
                temp = Point()
                temp.x = j / args.scale + args.offset_x
                temp.y = (img_arr.shape[0] - i) / args.scale + args.offset_y
                temp.z = -0.5 / args.scale
                tempMarker.points.append(temp)
    tempMarker.header.stamp = rospy.Time.now()
    pub_marker.publish(tempMarker)
    while not rospy.is_shutdown():
    #     pub_marker.publish(tempMarker)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        pub_convert()
    except rospy.ROSInterruptException:
        pass

