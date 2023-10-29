#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import numpy as np

roll = pitch = yaw = 0.0
target = 0
kp=0.5

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print(yaw)

def callback_angle(msg): 
    global target
    target = np.float(msg.data)
    print(target)


rospy.init_node('rotate_robot')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
sub_target_angle = rospy.Subscriber ('/angle', String, callback_angle)

r = rospy.Rate(10)
command =Twist()

while not rospy.is_shutdown():
    #quat = quaternion_from_euler (roll, pitch,yaw)
    #print quat
    target_rad = target*math.pi/180

    result = kp  * (target_rad - yaw)
    #result = np.sign(pre_result)

    if result>0.15:
        result = 0.2
    elif result<-0.15:
        result = -0.2
    

    command.angular.z = result
    pub.publish(command)
    #print("target={} current:{}", target,yaw)
    r.sleep()