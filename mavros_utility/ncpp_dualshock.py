#!/usr/bin/env python3

from __future__ import print_function
import argparse
import sys
import math

from tf.transformations import euler_from_quaternion

from mavros_msgs.srv import CommandBool, SetMode
from n_cpp.srv import NextStep, EliminateTriple, RayCast
import rospy

from std_msgs.msg import String, Time
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

position_mask = 0b101111111000
velocity_mask = 0b011111000111
rotate_mask = 0b011111111000

max_velocity = 2.0
max_yaw_rate = 1.570796
takeoff_height = 1.5

parser = argparse.ArgumentParser(description='NCPP control arg')
parser.add_argument('--path-type', type=int, default=0,
                    help='an integer for determine the path type(0:raw path, 1:raw traj, 1:VIO traj, default:0)')
parser.add_argument('--time-thresh', type=float, default=3.0,
                    help='path generation threshold(default: 3.0)')
parser.add_argument('--v-max', type=float, default=2.5,
                    help='max translational speed(default: 2.5)')
parser.add_argument('--yaw-max', type=float, default=0.3,
                    help='max yaw rate(default: 0.3)')
args = parser.parse_args()
pathType = args.path_type
genTimeThresh = args.time_thresh
v_max = args.v_max
yaw_max = args.yaw_max
print("-----------Control Args-----------")
print("  pathType     :", pathType)
print("  genTimeThresh:", genTimeThresh)
print("  max speed    :", v_max)
print("  max yaw rate :", yaw_max)
print("----------------------------------")

# mode : 'wait', 'move', 'takeoff', 'rotate', 'rotating', 'hover', 'pathStart', 'expStart', 'follow', 'analog'
ctrl_mode   = 'wait'
escape = False
stayCount = 0

state       = State()
ext_state   = ExtendedState()

joy_data    = Joy()

pose            = PoseStamped()
hover           = PositionTarget()
rotate_time     = Time()
path_start_time = Time()

last_ref = []
# curr_ref = []
local_path = Path()

wait = False

hover.coordinate_frame = 1;
hover.type_mask = position_mask;

def sub_setup():
    joy_sub            = rospy.Subscriber("/joy", Joy, joy_callback)
    state_sub          = rospy.Subscriber("/mavros/state", State, state_callback)
    ext_state_sub      = rospy.Subscriber("/mavros/extended_state", ExtendedState, ext_state_callback)
    local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_position_callback)

def move(control_msg):
    control_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    control_pub.publish(control_msg)

def takeoff():
    arming_srv  = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    arm_result  = arming_srv(True)
    mode_srv    = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    mode_result = mode_srv(0,"OFFBOARD")

def rotate():
    global yaw_max
    ctrl_msg = PositionTarget()
    ctrl_msg.type_mask  = rotate_mask
    ctrl_msg.position   = hover.position
    ctrl_msg.yaw_rate   = yaw_max
    return ctrl_msg

def nextStep(escape):
    next_srv    = rospy.ServiceProxy('/n_path/next_step', NextStep)
    next_result = next_srv(escape)
    return next_result.ref_triple, next_result.raw_path

def eliminateTriple(target):
    elimi_srv    = rospy.ServiceProxy('/n_octomap/eliminate_tri', EliminateTriple)
    elimi_result = elimi_srv(target)
    return elimi_result.result

def rayCast(start, end):
    ray_srv    = rospy.ServiceProxy('/n_octomap/ray_cast', RayCast)
    ray_result = ray_srv(start, end)
    return ray_result.visible

def requirePath():
    global last_ref
    global local_path
    global ctrl_mode
    global escape 
    global stayCount
    if stayCount>1:
        escape=True
        stayCount=0
    rospy.loginfo("escape %d",escape)
    curr_ref, local_path = nextStep(escape)
    escape = False
    if(len(local_path.poses) == 1):
        stayCount = stayCount+1
        ctrl_mode = 'get_surround'
        rospy.loginfo("stayCount:  %d",stayCount)
    if(not len(last_ref) == 0):
        eliminateTriple(last_ref)
    last_ref = curr_ref

def joy_callback(data): # self, 
    global joy_data
    global ctrl_mode
    global local_path
    global pathType

    #joy_data.axes[]
    #joy_data.buttons[]

    joy_data = data

    if joy_data.buttons[0] == 1:
        ctrl_mode = 'hover'
    elif joy_data.buttons[1] == 1:
        if ext_state.landed_state == ExtendedState().LANDED_STATE_ON_GROUND:
            rospy.loginfo("attempting takeoff")
            ctrl_mode = 'takeoff'
        else:
            rospy.loginfo("already flying : %lf", pose.pose.position.z)
    elif joy_data.buttons[2] == 1:
        ctrl_mode = 'rotate'
    elif joy_data.buttons[3] == 1:
        ctrl_mode = 'pathStart'
        rospy.loginfo("pathStart")
        requirePath()
        rospy.loginfo("local_path size: %d", len(local_path.poses))
    elif joy_data.buttons[10] == 1:
        ctrl_mode = 'analog'

def diff_distance(curr, dest):
    dist = ((curr.pose.position.x - dest.pose.position.x)**2+(curr.pose.position.y - dest.pose.position.y)**2+(curr.pose.position.z - dest.pose.position.z)**2)**0.5
    return dist

def calc_yaw(data):
    quat = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    (R, P, yaw) = euler_from_quaternion(quat)
    return yaw
    
def test_path_arrive(pose, local_path):
    global ctrl_mode
    if(diff_distance(pose, local_path.poses[0]) < v_max/1.0 and abs(calc_yaw(pose)-calc_yaw(local_path.poses[0]))<0.1):
        local_path.poses.remove(local_path.poses[0])
        rospy.loginfo("arrived!")
    if(len(local_path.poses)==0):
        # ctrl_mode = 'get_surround'
        return True
    else:
        rospy.loginfo("destination path: (%f, %f, %f)", local_path.poses[0].pose.position.x, local_path.poses[0].pose.position.y, local_path.poses[0].pose.position.z)
        return False

def set_vel_ctrl(curr, dest): 
    vel_msg = PositionTarget()
    vel_msg.coordinate_frame = 1
    dist = diff_distance(curr, dest)
    diff_yaw = calc_yaw(dest) - calc_yaw(curr)
    while not -3.141592<diff_yaw<3.141592:
        if(diff_yaw>3.141592):
            diff_yaw = diff_yaw - 2.0*3.141592 
        elif(diff_yaw<-3.141592):
            diff_yaw = diff_yaw + 2.0*3.141592
    print("diff_yaw: ", diff_yaw)
    print("dist    : ", dist)
    vel_msg.yaw_rate = yaw_max * diff_yaw
    thresh1 = v_max/1.0
    thresh2 = v_max/3.0
#    thresh2 = v_max/12.5
#    thresh2 = v_max/1.0
    if(thresh2 < dist < thresh1):
        vel_msg.type_mask = velocity_mask
        vel_msg.velocity.x = v_max * (dest.pose.position.x - curr.pose.position.x)/thresh1
        vel_msg.velocity.y = v_max * (dest.pose.position.y - curr.pose.position.y)/thresh1
        vel_msg.velocity.z = v_max * (dest.pose.position.z - curr.pose.position.z)/thresh1
        vel_msg.yaw_rate = yaw_max * diff_yaw
    elif(thresh2 > dist):
        vel_msg.type_mask = position_mask
        vel_msg.position.x = dest.pose.position.x
        vel_msg.position.y = dest.pose.position.y
        vel_msg.position.z = dest.pose.position.z
        vel_msg.yaw = calc_yaw(dest)
    else:
        vel_msg.type_mask = velocity_mask
        vel_msg.velocity.x = v_max * (dest.pose.position.x - curr.pose.position.x)/dist
        vel_msg.velocity.y = v_max * (dest.pose.position.y - curr.pose.position.y)/dist
        vel_msg.velocity.z = v_max * (dest.pose.position.z - curr.pose.position.z)/dist
        vel_msg.yaw_rate = yaw_max * diff_yaw
    return vel_msg
        
def state_callback(data): 
    global state
    state = data

def ext_state_callback(data): 
    global ext_state
    ext_state = data

def local_position_callback(data): 
    global pose
    global hover
    global log
    global local_path
    global rotate_time
    global path_start_time
    global max_velocity
    global takeoff_height
    global position_mask
    global velocity_mask
    global rotate_mask
    global ctrl_mode
    global pathType
    global genTimeThresh

    setpoint_msg = PositionTarget()
    setpoint_msg.header.stamp = rospy.Time.now()
    setpoint_msg.coordinate_frame = 1

    hover.header.stamp = rospy.Time.now()

    pose = data

    current_yaw = calc_yaw(data)

    if(ctrl_mode == 'analog'):
        setpoint_msg.coordinate_frame = 8
        setpoint_msg.type_mask = velocity_mask
        setpoint_msg.velocity.x = max_velocity * joy_data.axes[4]
        setpoint_msg.velocity.y = max_velocity * joy_data.axes[3]
        setpoint_msg.velocity.z = max_velocity * joy_data.axes[1]
        setpoint_msg.yaw_rate = max_yaw_rate * joy_data.axes[0]
    elif(ctrl_mode == 'takeoff'):
        setpoint_msg.type_mask  = position_mask
        setpoint_msg.position   = pose.pose.position
        setpoint_msg.position.z = takeoff_height
        setpoint_msg.yaw        = current_yaw
        hover = setpoint_msg
        takeoff()
        ctrl_mode = 'wait'
    elif(ctrl_mode == 'hover'):
        rospy.loginfo("hover")
        hover.type_mask  = position_mask
        hover.position   = data.pose.position
        hover.yaw        = current_yaw
        setpoint_msg = hover
        ctrl_mode = 'wait'
    elif(ctrl_mode == 'rotate'):
        rospy.loginfo("rotate")
        rotate_time = rospy.Time.now()
        hover.type_mask  = position_mask
        hover.position   = data.pose.position
        hover.yaw        = current_yaw
        setpoint_msg = rotate()
        ctrl_mode = 'rotating'
    elif(ctrl_mode == 'rotating'):
        setpoint_msg = rotate()
        howLong = rospy.Time.now() - rotate_time 
        if(howLong.to_sec() > math.ceil(2.0*3.141592/setpoint_msg.yaw_rate)):
            rospy.loginfo("rotate end")
            ctrl_mode = 'wait'
    elif(ctrl_mode == 'get_surround'):
        rospy.loginfo("get_surround")
        rotate_time = rospy.Time.now()
        hover.type_mask  = position_mask
        hover.position   = data.pose.position
        hover.yaw        = current_yaw
        setpoint_msg = rotate()
        # requirePath()
        ctrl_mode = 'surround_rotating'
    elif(ctrl_mode == 'surround_rotating'):
        setpoint_msg = rotate()
        # setpoint_msg.yaw_rate = 1.1
        howLong = rospy.Time.now() - rotate_time 
        if(howLong.to_sec() > math.ceil(2.0*3.141592/setpoint_msg.yaw_rate)):
            requirePath()
            rospy.loginfo("surround_rotating end")
            ctrl_mode = 'pathStart'
    elif(ctrl_mode == 'wait'):
        setpoint_msg = hover
    elif(ctrl_mode == 'pathStart'):
        path_start_time = rospy.Time.now()
        hover.type_mask  = position_mask
        hover.position   = data.pose.position
        hover.yaw        = current_yaw
        setpoint_msg = hover
        log = False
        ctrl_mode = 'follow'
    elif(ctrl_mode == 'follow'):
        pathGenTime = rospy.Time.now()-path_start_time
        setpoint_msg = hover
#        if(len(local_path.poses) != 0):
#            if(log ==False):
#                rospy.loginfo("path generation time: %f", pathGenTime.to_sec())
#                log = True
        if(len(local_path.poses) == 0 and pathGenTime.to_sec() > genTimeThresh):
            rospy.loginfo("path not generated....")
            rospy.loginfo("retry path generation!")
            ctrl_mode = 'get_surround'
        elif(len(local_path.poses) == 0 and pathGenTime.to_sec() < genTimeThresh):            
            rospy.loginfo("John Bough %f", pathGenTime.to_sec())
            setpoint_msg = hover
        elif(len(local_path.poses)!=0):
            if(pathType==0):
                termination_condition = test_path_arrive(data, local_path)
                if(termination_condition):
                    setpoint_msg = hover
                    rospy.loginfo("DONE")
                    # ctrl_mode = 'get_surround'
                    ctrl_mode = 'pathStart'
                    requirePath()
                else:
                    # rospy.loginfo("Now on progress")
                    setpoint_msg = set_vel_ctrl(data, local_path.poses[0])
#                    setpoint_msg.type_mask  = position_mask
#                    setpoint_msg.position = local_path.poses[0].pose.position
#                    setpoint_msg.yaw = calc_yaw(local_path.poses[0])
            elif(pathType==1):
                setpoint_msg = hover
            elif(pathType==2):
                setpoint_msg = hover
    move(setpoint_msg)

if __name__ == '__main__':
    # global pose
    # global setpoint_msg
    rospy.init_node('dualshock_UAV', anonymous=True)
    rate = rospy.Rate(30)
    sub_setup()

    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('Node terminated by pressing Ctrl+C!')
            sys.exit(0)
