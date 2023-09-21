#!/usr/bin/env python2
import rospy
import numpy as np
#import time
#import random
import os,sys
#from DQN_functions.mobile_control import deep_mobile_control
#from DQN_functions.ReplayMemory import ReplayMemory

from mobile_control import deep_mobile_control_new
from ReplayMemory import ReplayMemory


#from keras import Sequential
from keras.models import load_model

epsilon =0
min_epsilon=0.1
epsilon_decay=0.999
vels=[[1,0],[0.5,1],[0.5,-1],[0,1],[0,-1],[-0.8,0]]
gamma=0.9
legal_actions=len(vels)
reward_temp=0
batch_size=32
rospy.init_node('Mobile_Control',anonymous=True)
Robot = deep_mobile_control_new()
memory_size=1000
memory=ReplayMemory(memory_size)

model1=load_model(os.path.dirname(os.path.realpath(sys.argv[0]))+'/jetson_model_1.h5')
model2=load_model(os.path.dirname(os.path.realpath(sys.argv[0]))+'/jetson_model_2.h5')
def main():
    test_model()
    #train_model()
def perform_action(a):
    print('action number:',a)
    Robot.spd_comd.data=a
    Robot.spd_command_publisher.publish(Robot.spd_comd)
def choose_action(s):
    global epsilon,min_epsilon
        #epsilon greedy. to choose random actions initially when Q is all zeros
    if np.random.random()<epsilon:
        action_index=np.random.randint(0,legal_actions)
        a =action_index
        if epsilon>min_epsilon:
            epsilon = epsilon*epsilon_decay
        print('Took random action')
    else:
        Q = (model1.predict(np.array([s]))+model2.predict(np.array([s])))/2
        
        a =int(np.argmax(Q))
	#a=1	
        print(Q)
    return a
def get_state():
    s_temp=Robot.data_laser
    ang_sp=Robot.odom_data.angular.z
    lin_sp=Robot.odom_data.linear.x
    s_temp=np.append(s_temp,[ang_sp,lin_sp])
    return s_temp
def append_hist(a,value):
    with open(a, 'a') as f:
        f.write("%s\n" % value)
def test_model():
    global epsilon
    epsilon=0.1
    while not rospy.is_shutdown():
        s=get_state()
        a=choose_action(s)
        perform_action(a)
        try:           
            rospy.sleep(0.1)
        except:
            pass
def store_transition(delay_time):
        global reward_temp
        s=get_state()
        a=choose_action(s)
        perform_action(a)
        try:
            rospy.sleep(delay_time)
        except:
            pass    
        s1=get_state()
        reward=max(Robot.odom_data.linear.x,0)*30+0.5*(s1[0]+s1[4])+(s1[1]+s1[3])+2*s1[2]-10
        experience=(s,reward,a,s1)
        memory.append(experience)
        
        reward_temp=reward_temp+reward

def optimize_model():
    global reward_temp
    if memory.__len__()>batch_size: 
        size=batch_size
    else:
        size=memory.__len__()
    batches=memory.sample(size)
    states= np.array([batch[0] for batch in batches])
    rewards= np.array([batch[1] for batch in batches])
    actions= np.array([batch[2] for batch in batches])
    new_states= np.array([batch[3] for batch in batches])
    if np.random.random()<0.5:
        temp_model1=model1
        temp_model2=model2
    else:
        temp_model1=model2
        temp_model2=model1
    #print(actions)
    Qs =temp_model1.predict(states)
    next_state_Qs = temp_model1.predict(new_states)
    next_state_actions=np.argmax(next_state_Qs,axis=1)

    for i in range(len(rewards)):
        action_index=vels.index(list(actions[i]))
        next_state_action_index=next_state_actions[i]
    
        next_state_value=np.squeeze(temp_model2.predict(new_states[i].reshape(1,-1)))[next_state_action_index]
        Qs[i][action_index]= rewards[i]+gamma*next_state_value

    history=temp_model1.fit(states,Qs,batch_size=len(batches),callbacks=[early_stop],epochs =epochs )
    append_hist('loss_history.txt',history.history['loss'][-1])

if __name__== "__main__":
    main()
