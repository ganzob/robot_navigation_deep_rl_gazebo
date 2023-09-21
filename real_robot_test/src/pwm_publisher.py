#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 23 14:37:13 2019

@author: ganzobat
"""
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
publish_rate=2  #2Hz
linear_speed=0
angular_speed=0
speed_command=6
def command_callback(msg):
    global speed_command
    speed_command=msg.data
    #print('spd:',speed_command)
#def speed_callback(msg):
##    linear_speed=msg.linear.x
#    angular_speed=msg.angular.z
#def odom_callback(msg1):
#    odom_data=msg1.twist.twist

#command_sub=rospy.Subscriber('/cmd_vel',Twist,callback=speed_callback)
#speed_sub=rospy.Subscriber('/odom',Odometry,callback=odom_callback)
command_sub=rospy.Subscriber('/spd_command',Int16,callback=command_callback)
pub = rospy.Publisher('/pwms', Int16MultiArray, queue_size=1)
rospy.init_node('speed_esdfstimator', anonymous=True)
rate = rospy.Rate(publish_rate) 
pwms = Int16MultiArray()
pwms.data = [0,0]

Kp=10
Ki=0
Kd=0

linear_speed=0   #desired linear speed from DQN
angular_speed=0  #desired angular speed from DQN
pwm_list=[[120,120],[80,60],[60,80],[80,-80],[-80,80],[-80,-80],[0,0]]
#vels=[[0.8,0],[0.3,0.7],[0.3,-0.7],[0,0.7],[0,-0.7],[-0.5,0]]

def calculate_pwm():
    global pwm_list,pwms,speed_command    
    pwms.data[0]=pwm_list[speed_command][0]        
    pwms.data[1]=pwm_list[speed_command][1]        

def main():
    while not rospy.is_shutdown():
        calculate_pwm()
        #print('pwms:',pwms.data)
	#test_run()
        pub.publish(pwms)
        rate.sleep()
def stop():
    pwms.data = [0,-0]
    pub.publish(pwms)
def test_run():
    a=0
    while (1):
        if a==0:
            pwms.data = [100,0]
            pub.publish(pwms)
            a=1
        elif a==1:
            pwms.data = [-50,50]
            pub.publish(pwms)
            a=0                
        rate.sleep()        

#test_run()
#stop()
if __name__ == '__main__':
    main()
