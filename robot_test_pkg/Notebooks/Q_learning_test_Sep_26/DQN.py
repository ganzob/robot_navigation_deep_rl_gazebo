import numpy as np
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
from tqdm import tqdm
import random
import keyboard
#import Mobile_Control
from collections import deque
from keras import Sequential
from keras.layers import Dense
from keras.optimizers import Adam

cmd_vel="/cmd_vel"
laser_topic="/scan"
odom_topic="/odom"
imu_topic = "/imu"

#Hyperparameters
gamma = 0.9
lr =0.5
lr_decay=0.999
num_episodes =600000
epsilon =0.5
epsilon_decay =0.999999

actions=[[1,0],[0.5,1],[0.5,-1],[0,1],[0,-1],[-0.8,0]]
legal_actions=len(actions)
s=np.zeros([3,3,3,3,3])   
#Q=np.zeros((3,3,3,3,3,len(linear_actions),len(angular_actions)))
#Q=np.zeros((s.size,legal_actions))
Q=np.load('Q-table.npy')
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

batch_size=100
memory_size=10000
memory = deque(maxlen=memory_size)
Robot.reset()

#experience replay
s=Robot.data_laser
s=numpy.append(s,[odom_data.linear.x,odom_data.angular.z])
for _ in range(memory_size):
    a = actions[np.random.randint(0,legal_actions)]
    Robot.vel.linear.x=a[0]
    Robot.vel.angular.z=a[1]
    Robot.cmd_vel.publish(Robot.vel)
    time.sleep(0.1)
    s1=Robot.data_laser
    s1=numpy.append(s1,[odom_data.linear.x,odom_data.angular.z])
    r=max(Robot.odom_data.linear.x,0)
    experience =(s,r,a,s1)
    memory.append(experience)
    s=Robot.data_laser
    s=numpy.append(s,[odom_data.linear.x,odom_data.angular.z])
    
model = Sequential()
model.add(Dense(7,activation='sigmoid',input_dim=7))
model.add(Dense(6,activation='sigmoid'))
model.add(Dense(legal_actions,activation='linear'))
model.compile(loss='mean_absolute_error',optimizer=Adam(lr=0.01))
model.summary()    


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
   # train_model()   
    test_model()

def train_model():
    global epsilon
    process_bar = tqdm(range(num_episodes))
    for i in process_bar:
        if(i%1000==0):
            print('eps:',epsilon)
            if(lr>0.2):
                lr=lr*lr_decay
        if(abs(Robot.imu_x)>0.3 or abs(Robot.imu_y)>0.3):
            Robot.reset()
        if(abs(Robot.odom_data.linear.x)>4 or abs(Robot.odom_data.angular.z)>4):
            Robot.reset()
        s=Robot.data_laser
           #state index
           #Robot.pause()
        s0=(int)(s[0]*81+s[1]*27+s[2]*9+s[3]*3+s[4])
        #epsilon greedy. to choose random actions initially when Q is all zeros
        if np.random.random()<epsilon:
            action_index=np.random.randint(0,legal_actions)
            a =actions[action_index]
            a = actions[action_index]        
            epsilon = epsilon*epsilon_decay
            #print('Took random action')
        else:
            Q = model.predict(np.array([s]))
            a =actions[np.argmax(Q)]
        Robot.vel.linear.x = a[0]
        Robot.vel.angular.z=a[1]
        Robot.cmd_vel.publish(Robot.vel)
        time.sleep(0.1)
        ss=Robot.data_laser
        # next state index
        reward=max(Robot.odom_data.linear.x,0)
        experience=(s,r,a,s1)
        memory.append(experience) 

        batches=random.sample(memory,batch_size)
        states= np.array([batch[0] for batch in batches])
        rewards= np.array([batch[1] for batch in batches])
        actions= np.array([batch[2] for batch in batches])
        new_states= np.array([batch[3] for batch in batches])

        Qs =model.predict(states)
        new_Qs = model.predict(new_states)
        for i in range(len(rewards)):
            action_index=vels.index(list(actions[i]))
                      #action_index=list(vels).index(actions[i])
            Qs[i][action_index]= Qs[i][action_index]+ lr*(rewards[i]+gamma*np.max(new_Qs[i]))
    
    model.fit(states,Qs,verbose=0)


        Q[s0,action_index]= (1-lr)*Q[s0,action_index]+ lr*(reward+gamma*np.max(Q[j,:]))
        if (i%1000==0):  #Save model in every 1000 iterations
            np.save('Q-table.npy',Q)
    np.save('Q-table.npy',Q)


def test_model():
    Q=np.load('Q-table.npy')
    while 1:
        #if keyboard.is_pressed('q'):
         #   break
        s=Robot.data_laser
        s0=(int)(s[0]*81+s[1]*27+s[2]*9+s[3]*3+s[4])
        action_index=np.argmax(Q[s0,:])
        a =actions[action_index]
        Robot.vel.linear.x = a[0]
        Robot.vel.angular.z=a[1]
        Robot.cmd_vel.publish(Robot.vel)    

if __name__== "__main__":
    main()