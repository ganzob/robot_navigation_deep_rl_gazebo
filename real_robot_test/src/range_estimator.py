#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 22 14:37:13 2019

@author: ganzobat
"""

from rosserial_arduino.msg import Adc   # Datatype to read IR sensor value from arduino
from sensor_msgs.msg import LaserScan   # Dateatype to publish IR sensor ranges 
from collections import deque   # will use it to calculate moving averages 
import rospy
import numpy as np

ir_ranger_adc_topic='/adc'
real_range='/scan'

rospy.init_node('range_estimator')
n_readings=5
laser_frequency=100
r=rospy.Rate(laser_frequency)
moving_avg=10
memory = deque(maxlen=moving_avg)
b=5127
m=-1.007  # range=b*adc^m

class IRRange_Laser():
    def __init__(self):
        self.range=rospy.Publisher(real_range, LaserScan, queue_size=2)
        self.scan = LaserScan()
        self.scan.header.frame_id = 'laser_frame'
        self.scan.angle_min = -1.57
        self.scan.angle_max = 1.57
        self.scan.angle_increment = (self.scan.angle_max-self.scan.angle_min) / n_readings
        self.scan.time_increment = (1.0 / laser_frequency) / (n_readings)
        self.scan.range_min = 0.0
        self.scan.range_max = 0.6
    def publish_ranges(self,ranges):
        self.scan.header.stamp = rospy.Time.now()
        self.scan.ranges=ranges
        self.range.publish(self.scan)

class ADC():
    def __init__(self):
        self.ir_adc=rospy.Subscriber(ir_ranger_adc_topic, Adc, callback=self.adc_callback)
        self.values=[]
    def adc_callback(self,adc):
        self.q_temp=[]
        self.a=adc
        self.q_temp.append(self.a.adc0)
        self.q_temp.append(self.a.adc1)
        self.q_temp.append(self.a.adc2)
        self.q_temp.append(self.a.adc3)
        self.q_temp.append(self.a.adc4)
        self.values=self.q_temp

adc=ADC()
ranges=IRRange_Laser()

def main():
    while not rospy.is_shutdown():
        try:
            #adc_values=adc.values                      
            memory.append(adc.values)
            a=np.array(memory)
       	    if len(a)<8:
    		    a=b*pow(np.mean(a,axis=0),m)
    	    else:
                q=np.sort(a,axis=0)
                a=0.01*b*pow(np.mean(q[2:8]+1e-5,axis=0),m)
                
            for i in range(len(a)):
                if a[i]>0.6:
   		            a[i]=0.6

            ranges.publish_ranges(a)         
            r.sleep()
        except:
            pass
if __name__== "__main__":
    main()    
    
