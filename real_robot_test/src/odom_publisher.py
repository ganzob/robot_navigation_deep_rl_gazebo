#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 23 19:40:27 2019

@author: luu
"""

import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
base_diameter=0.24
left_speed=2
right_speed=2
def imu_callback(msg):
    pass
def speed_callback(msg):
    global left_speed, right_speed
    left_speed=msg.data[0]
    right_speed=msg.data[1]
    #print(msg.data)
speed_subscriber=rospy.Subscriber('speeds',Float32MultiArray,callback=speed_callback)
odom_pub=rospy.Publisher('/odom',Odometry,queue_size=1)
imu_sub=rospy.Subscriber('/imu',Imu,callback=imu_callback)
rospy.init_node('odom_publisher')
r=rospy.Rate(100)

odom=Odometry()
odom.header.stamp = rospy.Time.now()
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"

odom.twist.twist.angular.x=0
odom.twist.twist.angular.y=0
odom.twist.twist.angular.z=0

odom.twist.twist.linear.x=0
odom.twist.twist.linear.y=0
odom.twist.twist.linear.z=0

odom.pose.pose.position.x=0
odom.pose.pose.position.y=0
odom.pose.pose.position.z=0

def main():
    while not rospy.is_shutdown():        
        try:
            odom.header.stamp = rospy.Time.now()

            odom.twist.twist.linear.x=(left_speed+right_speed)/2
            #print('fadf')
            odom.twist.twist.angular.z=(left_speed-right_speed)/base_diameter
            odom_pub.publish(odom)
            r.sleep()
        except KeyboardInterrupt:
            break
        except:
            continue
#        rospy.spinOnce()
        

if __name__ == '__main__':
    main()
