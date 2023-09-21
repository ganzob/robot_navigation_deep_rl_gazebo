import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
#from move_base_msgs.msg import MoveBaseGoal
cmd_vel="/cmd_vel"
laser_topic="/scan"
odom_topic="/odom"
imu_topic = "/imu"
goal_topic="move_base_simple/goal/pose/position"
class mobile_control():
    def __init__(self):
        self.cmd_vel = rospy.Publisher(cmd_vel,Twist,queue_size =1)
        self.subscriber = rospy.Subscriber(laser_topic,LaserScan,callback=self.laser_callback)
        self.subscriber_odo=rospy.Subscriber(odom_topic,Odometry,callback=self.odom_callback)
        self.subscriber_imu=rospy.Subscriber(imu_topic, Imu, callback=self.imu_callback)
        self.reset = rospy.ServiceProxy("/gazebo/reset_simulation",Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics",Empty)
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics",Empty)
        self.i =0
        self.count =0
        self.vel=Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0 
    def odom_callback(self,msg1):
        self.odom_data=msg1.twist.twist
        self.odom_pose=msg1.pose.pose.position

    def laser_callback(self, msg):
        self.laser_data=msg.ranges
        self.data=np.array(self.laser_data)
        self.laser_mask=np.logical_or(self.data>2, self.data<0)
        self.range_angels=np.arange(len(self.laser_data))
        self.i=0
        self.range_angels2=self.range_angels[self.laser_mask]
        for self.i in (self.range_angels2):
            self.data[self.i]=2
        self.data_laser=np.round(self.data[:])
    def imu_callback(self, msg_imu):
        self.imu_x=msg_imu.orientation.x
        self.imu_y=msg_imu.orientation.y
class deep_mobile_control():
    def __init__(self):
        self.cmd_vel = rospy.Publisher(cmd_vel,Twist,queue_size =1)
        self.subscriber = rospy.Subscriber(laser_topic,LaserScan,callback=self.laser_callback)
        self.subscriber_odo=rospy.Subscriber(odom_topic,Odometry,callback=self.odom_callback)
        self.subscriber_imu=rospy.Subscriber(imu_topic, Imu, callback=self.imu_callback)
        self.reset = rospy.ServiceProxy("/gazebo/reset_simulation",Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics",Empty)
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics",Empty)
        self.i =0
        self.count =0
        self.vel=Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0 
    def odom_callback(self,msg1):
        self.odom_data=msg1.twist.twist
        self.odom_pose=msg1.pose.pose.position

    def laser_callback(self, msg):
        self.laser_data=msg.ranges
        self.data=np.array(self.laser_data)
        self.laser_mask=np.logical_or(self.data>2, self.data<0)
        self.range_angels=np.arange(len(self.laser_data))
        self.i=0
        self.range_angels2=self.range_angels[self.laser_mask]
        for self.i in (self.range_angels2):
            self.data[self.i]=2
        self.data_laser=self.data[:]
    def imu_callback(self, msg_imu):
        self.imu_x=msg_imu.orientation.x
        self.imu_y=msg_imu.orientation.y
    def check_robot(self):
        if(abs(self.imu_x)>0.2 or abs(self.imu_y)>0.2):
            return True#self.reset()
        elif(abs(self.odom_pose.x)>10 or abs(self.odom_pose.y)>10):
            return True#self.reset()
        elif(abs(self.odom_data.angular.z)>10 or abs(self.odom_data.linear.x)>10):
            return True#self.reset()
        else:
            return False        
class deep_mobile_control_goal(deep_mobile_control):
    def __init__(self):
        super().__init__()                   
        self.goal_topic=rospy.Publisher(goal_topic, PoseStamped,queue_size=1)
    def isFinal(self,goal_x,goal_y,threshold):
        if(abs(goal_x-self.odom_pose.x)<threshold and abs(self.odom_pose.y-goal_y)<threshold):
            return True
        else:
            return False    