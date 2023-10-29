#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import rospy
from std_msgs.msg import Float64

def get_rotation (msg):
    global position_x, position_y
    #position = msg.pose.pose.position

    position_x = float(msg.pose.pose.position.x)
    position_y = float(msg.pose.pose.position.y)

    #print("X = ", position_x, "Y = ", position_y)

def talker_angle():

    pub_angle = rospy.Publisher('/angle', String, queue_size=10)
    pub_move = rospy.Publisher('/move_forward', String, queue_size=10)
    sub = rospy.Subscriber ('/odom', Odometry, get_rotation)

    rospy.init_node('angle_move_talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #pub_move.publish("0")
        #

        position = str(input("angle = "))
        print(position)
        rospy.loginfo(position)
        pub_angle.publish(position)
        rate.sleep()

        # 1 or 2 or 3 or 4 squeres
        position_xy = str(input("forward = "))

        rospy.loginfo(position_xy)
        
        if position_xy == "1":
            pub_move.publish("1")
            print("here")

        elif position_xy == "2":
            pub_move.publish("2")

        elif position_xy == "3":
            pub_move.publish("3")

        elif position_xy == "4":
            pub_move.publish("4")
           
        rate.sleep()
        #pub_move.publish("0")

# def talker_forward():
#     pub = rospy.Publisher('/move_forward', String, queue_size=10)
#     rospy.init_node('talker_forward', anonymous=True)
    
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         position = str(input("forward = "))
#         rospy.loginfo(position)
#         pub.publish(position)
#         rate.sleep()
        

if __name__ == '__main__':
    try:
        talker_angle()
        # talker_forward()
    except rospy.ROSInterruptException:
        pass