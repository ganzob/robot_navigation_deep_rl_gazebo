import numpy as np
import rospy
import sys
from matplotlib import pyplot as plt
import time
from tqdm import tqdm
import random
from mobile_control import deep_mobile_control
from collections import deque
from keras import Sequential
from keras.layers import Dense,Dropout
from keras.optimizers import Adam,SGD
from keras.models import load_model
#Hyperparameters
gamma = 0.9
lr =0.7
lr_decay=0.999
num_iterations =10000
epsilon =0.999
epsilon_decay =0.99999
min_epsilon=0.2
delay_time=0.05
vels=[[1,0],[0.5,1],[0.5,-1],[0,1],[0,-1],[-0.8,0]]
legal_actions=len(vels)
batch_size=512
memory_size=5000
memory = deque(maxlen=memory_size)
reward_temp=0
rospy.init_node('Mobile_Control',anonymous=True)
Robot = deep_mobile_control()
Robot.reset()
'''
optimizer=Adam(learning_rate=0.0001, beta_1=0.9, beta_2=0.999, amsgrad=False)

#optimizer=SGD(learning_rate=0.001,momentum=0.9)

model = Sequential()
model.add(Dense(200,activation='sigmoid',input_dim=7,kernel_initializer='random_uniform',use_bias='True'))
model.add(Dense(100,activation='sigmoid',kernel_initializer='random_uniform',use_bias='True'))
model.add(Dense(100,activation='sigmoid',kernel_initializer='random_uniform',use_bias='True'))
model.add(Dropout(0.5))
model.add(Dense(20,activation='sigmoid',kernel_initializer='random_uniform',use_bias='True'))
model.add(Dense(20,activation='sigmoid',kernel_initializer='random_uniform',use_bias='True'))

model.add(Dropout(0.4))
model.add(Dense(10,activation='sigmoid',kernel_initializer='random_uniform',use_bias='True'))
#model.add(Dense(6,activation='sigmoid',kernel_initializer='random_uniform',use_bias='True'))
model.add(Dense(legal_actions,activation='linear',kernel_initializer='random_uniform',use_bias='True'))
model.compile(loss='mean_absolute_error',optimizer=optimizer)
model.summary()
model.save('model1.h5')
'''
model=load_model('model1.h5')
target_model=load_model('model1.h5')

def main():    
    try:
        a=Robot.data_laser
        a=Robot.imu_x
        a=Robot.odom_data
        print(a)
    except:
        print("Error occured. Can't subscribe sensor info")
    #experience_replay()
    #train_model()   
    #plot_loss_history()
    #plot_reward()
    test_model()
def train_model():
    global lr,reward_temp
    process_bar = tqdm(range(num_iterations))
    for i in process_bar:
        if(i%1000==0):
            print('eps:',epsilon)
            if(lr>0.2):
                lr=lr*lr_decay
            model.save('model1.h5')
            target_model=load_model('model1.h5')
            #rAll_list.append(reward_temp/1000)
            append_hist('reward_hist.txt',reward_temp/1000)
            reward_temp=0
        check_robot()
        s=get_state()
        a=choose_action(s)
        take_action(a)
        time.sleep(delay_time)
        s1=get_state()
        reward=max(Robot.odom_data.linear.x,0)
        experience=(s,reward,a,s1)
        memory.append(experience) 
        batches=random.sample(memory,batch_size)
        states= np.array([batch[0] for batch in batches])
        rewards= np.array([batch[1] for batch in batches])
        actions= np.array([batch[2] for batch in batches])
        new_states= np.array([batch[3] for batch in batches])
        Qs =model.predict(states)

        next_state_Qs = target_model.predict(new_states)
        for i in range(len(rewards)):
            action_index=vels.index(list(actions[i]))
                    #action_index=list(vels).index(actions[i])
            #Qs[i][action_index]= Qs[i][action_index]+ lr*(rewards[i]+gamma*np.max(new_Qs[i]))
            Qs[i][action_index]= rewards[i]+gamma*np.max(next_state_Qs[i])
        #print('sta:',states.shape())
        history=model.fit(states,Qs,batch_size=128,verbose=2,epochs =10 )
        
        # Monitoring parts
        reward_temp=reward_temp+reward

        #loss_history.append(history.history['loss'][-1])
        #        if (i%1000==0):  #Save model in every 1000 iterations
        append_hist('loss_history.txt',history.history['loss'][-1])

    model.save('model1.h5')

def take_action(a):
    Robot.vel.linear.x = a[0]
    Robot.vel.angular.z=a[1]
    Robot.cmd_vel.publish(Robot.vel)
def choose_action(s):
    global epsilon,min_epsilon
        #epsilon greedy. to choose random actions initially when Q is all zeros
    if np.random.random()<epsilon:
        action_index=np.random.randint(0,legal_actions)
        a =vels[action_index]
        if epsilon>min_epsilon:
            epsilon = epsilon*epsilon_decay
        #print('Took random action')
    else:
        Q = model.predict(np.array([s]))
        a =vels[np.argmax(Q)]
    return a
def get_state():
    s_temp=Robot.data_laser
    ang_sp=Robot.odom_data.angular.z
    lin_sp=Robot.odom_data.linear.x
    s_temp=np.append(s_temp,[ang_sp,lin_sp])
    return s_temp
def test_model():
    epsilon=0.2
    while (1):
        s=get_state()
        #print(s)
        a=choose_action(s)
        take_action(a)
        time.sleep(0.05)    
def experience_replay():
    #experience replay
    for _ in range(memory_size):
        s=get_state()
        a = vels[np.random.randint(0,legal_actions)]
        Robot.vel.linear.x=a[0]
        Robot.vel.angular.z=a[1]
        Robot.cmd_vel.publish(Robot.vel)
        time.sleep(delay_time)
        s1=get_state()
        r=max(Robot.odom_data.linear.x,0)
        experience =(s,r,a,s1)
        memory.append(experience)
def check_robot():
        if(abs(Robot.imu_x)>0.1 or abs(Robot.imu_y)>0.1):
            Robot.reset()
        if(abs(Robot.odom_data.linear.x)>4 or abs(Robot.odom_data.angular.z)>4):
            Robot.reset()
def append_hist(a,value):
    with open(a, 'a') as f:
        f.write("%s\n" % value)
    
if __name__== "__main__":
    main()