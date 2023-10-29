#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState

from std_msgs.msg import String
#import math
import numpy as np

position_x = 0
left_encoder = 0
right_encoder = 0
result = 0
target_angle = 0
target = 0
kp=0.9

def get_rotation (msg):
    global position_x, position_y
    position = msg.pose.pose.position
    position_x = msg.pose.pose.position.x
    position_y = msg.pose.pose.position.y
    #print("X ", position_x,"Y ", position_y)

def callback_move(msg): 
    global target
    target = np.float(msg.data)
    #print("target", target)

def callback_angle(msg): 
    global target_angle
    target_angle = np.float(msg.data)
    #print("target_angle ", target_angle)

def callback_encoders(msg):
    global left_encoder, right_encoder
    left_encoder = msg.left_encoder
    right_encoder = msg.right_encoder
    print("left encoder ", left_encoder, "right_encoder = ", right_encoder)

rospy.init_node('move_robot')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
sub_encoders = rospy.Subscriber ('/sensor_state', SensorState, callback_encoders)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
sub_target_move = rospy.Subscriber ('/move_forward', String, callback_move)
sub_target_angle = rospy.Subscriber ('/angle', String, callback_angle)
pub_move = rospy.Publisher('/move_forward', String, queue_size=10)

r = rospy.Rate(10)
command =Twist()

while not rospy.is_shutdown():
    #quat = quaternion_from_euler (roll, pitch,yaw)
    #print quat
    #target_rad = target*math.pi/180
    #print("target_angle = ", target_angle)
    # if int(target_angle) == 0 or int(target_angle) == 180 :
    #     result = kp  * (target - position_x)
  
    # elif int(target_angle) == 90 or int(target_angle) == -90:
    #     print("here")
    #     result = kp  * (target - position_y)

    if target > 0:

        start_left_encoder = left_encoder
        start_right_encoder = right_encoder

        while (left_encoder-start_left_encoder) < 16700 and (right_encoder-start_right_encoder) < 16700:
            command.linear.x = 0.08
            pub.publish(command)
        
        pub_move.publish("0")
        command.linear.x = 0.0
        pub.publish(command)
        target = 0
    #print("target={} current:{}", target,yaw)
    r.sleep()