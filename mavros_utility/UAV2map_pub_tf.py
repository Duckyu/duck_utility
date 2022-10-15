#!/usr/bin/env python3

import rospy
import sys

import tf
import tf_conversions
import tf2_ros

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from gazebo_msgs.msg import ModelStates

# pose = PoseStamped()
#pose = Pose()
br = tf2_ros.TransformBroadcaster()
st_br = tf2_ros.StaticTransformBroadcaster()

def sub_setup():
#    local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)
	model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

def static_tf_pub():
    global st_br
    static_transformStamped_up = TransformStamped()
    static_transformStamped_front = TransformStamped()
    #tf.transformations.quaternion_from_euler(0, -1.570796, 0)

    static_transformStamped_up.header.stamp = rospy.Time.now()
    static_transformStamped_up.header.frame_id = "base_link"
    static_transformStamped_up.child_frame_id = "up_camera_link"

    static_transformStamped_up.transform.translation.x = 0 
    static_transformStamped_up.transform.translation.y = 0 
    static_transformStamped_up.transform.translation.z = 0.05 

    quat = tf.transformations.quaternion_from_euler(0, 0, -1.570796)
    static_transformStamped_up.transform.rotation.x = quat[0]
    static_transformStamped_up.transform.rotation.y = quat[1]
    static_transformStamped_up.transform.rotation.z = quat[2]
    static_transformStamped_up.transform.rotation.w = quat[3]

    static_transformStamped_front.header.stamp = rospy.Time.now()
    static_transformStamped_front.header.frame_id = "base_link"
    static_transformStamped_front.child_frame_id = "front_camera_link"

    static_transformStamped_front.transform.translation.x = 0.1 
    static_transformStamped_front.transform.translation.y = 0 
    static_transformStamped_front.transform.translation.z = 0

    quat = tf.transformations.quaternion_from_euler(-1.570796, 0, -1.570796)
    static_transformStamped_front.transform.rotation.x = quat[0]
    static_transformStamped_front.transform.rotation.y = quat[1]
    static_transformStamped_front.transform.rotation.z = quat[2]
    static_transformStamped_front.transform.rotation.w = quat[3]

    st_br.sendTransform(static_transformStamped_up)
    st_br.sendTransform(static_transformStamped_front)
    return

def tf_pub(pose):
    global br
    tf_temp = TransformStamped()
    tf_temp.transform.translation = pose.position
    tf_temp.transform.rotation = pose.orientation
    tf_temp.header.stamp = rospy.Time.now()
    tf_temp.header.frame_id = 'map'
    tf_temp.child_frame_id = 'base_link'
    if (pose.orientation.x**2 + pose.orientation.y**2 + pose.orientation.z**2 + pose.orientation.w**2 > 0.95):
        br.sendTransform(tf_temp)
    return

def model_pub(pose):
    msg = PoseStamped()
    msg.pose.position = pose.position

    msg.pose.orientation = pose.orientation

    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    pub = rospy.Publisher("/UAV", PoseStamped, queue_size=10)
    pub.publish(msg)

#def local_position_callback(data):
#    global pose
#    pose = data.pose
#    return

def model_states_callback(data):
#    global pose
    pose = Pose()
    index = -1
    model_index = -1 
    for model in data.name:
        index+=1    	
        if model == "iris_front_up_depth_camera":
            model_index = index
            pose = data.pose[model_index]
            tf_pub(pose)
            model_pub(pose)
    if model_index == -1:
        print("check model name")
    return


if __name__ == '__main__':
    rospy.init_node('cvtPose2TF', anonymous=True)
    rate = rospy.Rate(50)
    sub_setup()

    while not rospy.is_shutdown():
        try:
            # rospy.loginfo("try")
            static_tf_pub()
#            model_pub()
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('Node terminated by pressing Ctrl+C!')
            sys.exit(0)
