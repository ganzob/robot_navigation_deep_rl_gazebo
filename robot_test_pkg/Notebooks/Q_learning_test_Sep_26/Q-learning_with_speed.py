import numpy as np
import rospy
import sys
import time
from tqdm import tqdm
import random
import keyboard
#import Mobile_Control

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

cmd_vel="/cmd_vel"
laser_topic="/scan"
odom_topic="/odom"
imu_topic = "/imu"

#Hyperparameters
gamma = 0.9
lr =0.4
lr_decay=0.999
num_episodes =500
epsilon =0.6
epsilon_decay =0.99999

actions=[[1,0],[0.5,1],[0.5,-1],[0,2],[0,-2],[-1.5,0]]
legal_actions=len(actions)
s=np.zeros([3,3,3,3,3,3,3])   
#Q=np.zeros((3,3,3,3,3,3
#Q=np.zeros((s.size,legal_actions))
Q=np.load('Q-table-sp.npy')
#import rospy
#import numpy as np
class Mobile_Control:
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


Robot = Mobile_Control()
Robot.reset()


def main():
    rospy.init_node('Mobile_Control',anonymous=True)
    try:
        a=Robot.data_laser
        a=Robot.imu_x
        a=Robot.odom_data
        print(a)
    except:
        print("Error occured. Can't subscribe sensor info")
    print(legal_actions)
    print(s.size)
    print(Q.shape)
    #train_model()   
    test_model()

def train_model():
    global epsilon
    global lr
    process_bar = tqdm(range(num_episodes))
    for i in process_bar:
        if(i%1000==0):
            print('eps:',epsilon)
            if(lr>0.2):               
                lr=lr*lr_decay
                print('lr:',lr)
        if(abs(Robot.imu_x)>0.3 or abs(Robot.imu_y)>0.3):
            Robot.reset()
        if(abs(Robot.odom_pose.x)>8 or abs(Robot.odom_pose.x)>8):
            Robot.reset()
        s=Robot.data_laser
        sp_ang=Robot.odom_data.angular.z
        if(sp_ang<-0.2):
            ang_sp=0
        elif(sp_ang>0.2):
            ang_sp=1
        else:
            ang_sp=2    
        sp_lin=Robot.odom_data.linear.x
        if(sp_lin<-0.2):
            lin_sp=0
        elif(sp_lin>0.2):
            lin_sp=1
        else:
            lin_sp=2        
        s=np.append(s,[ang_sp,lin_sp])
        print('s:',s)
           #state index
           #Robot.pause()
        s0=(int)(s[0]*81+s[1]*27+s[2]*9+s[3]*3+s[4]+243*s[5]+729*s[6])
        #epsilon greedy. to choose random actions initially when Q is all zeros
        if np.random.random()<epsilon:
            action_index=np.random.randint(0,legal_actions)
            epsilon = epsilon*epsilon_decay
            #print('Took random action')
        else:
            action_index=np.argmax(Q[s0,:])
        
        a =actions[action_index]
        Robot.vel.linear.x = a[0]
        Robot.vel.angular.z=a[1]
        Robot.cmd_vel.publish(Robot.vel)
        
        time.sleep(0.1)
        
        sp_ang=Robot.odom_data.angular.z
        if(sp_ang<-0.2):
            ang_sp=0
        elif(sp_ang>0.2):
            ang_sp=1
        else:
            ang_sp=2    
        sp_lin=Robot.odom_data.linear.x
        if(sp_lin<-0.2):
            lin_sp=0
        elif(sp_lin>0.2):
            lin_sp=1
        else:
            lin_sp=2
        ss=Robot.data_laser                
        ss=np.append(ss,[ang_sp,lin_sp])
        print('ss:',ss)
        # next state index
        j=(int)(ss[0]*81+ss[1]*27+ss[2]*9+ss[3]*3+ss[4]+ss[5]*243+ss[6]*729)
        reward=max(Robot.odom_data.linear.x,0)
        print('reward:',reward)
        Q[s0,action_index]= (1-lr)*Q[s0,action_index]+ lr*(reward+gamma*np.max(Q[j,:]))
        if (i%1000==0):  #Save model in every 1000 iterations
            np.save('Q-table-sp.npy',Q)
    np.save('Q-table-sp.npy',Q)
def test_model():
    Q=np.load('Q-table-sp.npy')
    
    while True:
    #if keyboard.is_pressed('q'):
        #   break
        if(abs(Robot.imu_x)>0.3 or abs(Robot.imu_y)>0.3):
            Robot.reset()
        if(abs(Robot.odom_pose.x)>8 or abs(Robot.odom_pose.x)>8):
            Robot.reset() 
        s=Robot.data_laser
        sp_ang=Robot.odom_data.angular.z
        if(sp_ang<-0.2):
            ang_sp=0
        elif(sp_ang>0.2):
            ang_sp=1
        else:
            ang_sp=2    
        sp_lin=Robot.odom_data.linear.x
        if(sp_lin<-0.2):
            lin_sp=0
        elif(sp_lin>0.2):
            lin_sp=1
        else:
            lin_sp=2
                
        s=np.append(s,[ang_sp,lin_sp])
        s0=(int)(s[0]*81+s[1]*27+s[2]*9+s[3]*3+s[4]+243*s[5]+729*s[6])
        if np.random.random()<0.2:
            action_index=np.random.randint(0,legal_actions)
            #a = actions[action_index]        
            #epsilon = epsilon*epsilon_decay
            print('Took random action')
        else:
            action_index=np.argmax(Q[s0,:])
        
        #action_index=np.argmax(Q[s0,:])
        a =actions[action_index]
        print(a)
        Robot.vel.linear.x = a[0]
        Robot.vel.angular.z=a[1]
        Robot.cmd_vel.publish(Robot.vel)    
    

if __name__== "__main__":
    main()