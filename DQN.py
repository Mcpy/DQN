from enum import auto
import tensorflow as tf
import math
import numpy as np
import random




class DQN():
    def __init__(self,input_num,hidden_info,output_num,lr=1e-6,path=None,autosave=False):
        self.modelpath=path
        self.autosave=autosave
        self.mean_rmse=0
        self.num_rmse=0
        if path is None:
            self.evaluateNN=self.getDeepNet(input_num,hidden_info,output_num)
            self.targetNN=self.getDeepNet(input_num,hidden_info,output_num)
            self.copyNN()
            self.modelpath="model"
        else:
            try:
                self.evaluateNN=tf.keras.models.load_model(path)
                self.targetNN=tf.keras.models.load_model(path)
            except:
                print('cannot load model')
                self.evaluateNN=self.getDeepNet(input_num,hidden_info,output_num)
                self.targetNN=self.getDeepNet(input_num,hidden_info,output_num)
                self.copyNN()
        self.optimizer=tf.keras.optimizers.Adam(lr)
        # self.lossmean=tf.keras.metrics.Mean('lossmean')


    def getDeepNet(self,input_num,hidden_info,output_num):
        model=tf.keras.Sequential()
        # input layer
        model.add(tf.keras.layers.Dense(input_num,activation='relu',input_shape=(input_num,)))
        for i in hidden_info:
            model.add(tf.keras.layers.Dense(i,activation='relu'))
        model.add(tf.keras.layers.Dense(output_num,activation='linear'))
        return model

    def copyNN(self):
        # for e_var,t_var in zip(self.evaluateNN.variables,self.targetNN.variables):
        #     t_var.assign(e_var)
        self.targetNN.set_weights(self.evaluateNN.get_weights())

    def train_step(self,transition,gamma=0.9,lr=1):
        # transition={"state":[],"action":0,"reward":0,"nextstate":[]}
        rmse=0
        with tf.GradientTape() as t:
            target_q=self.evaluateNN(transition['state']).numpy()[0]
            q_t=transition['reward']+gamma*tf.reduce_max(self.targetNN(transition['nextstate'])) #dqn or ddqn
            target_q[transition['action']]=lr*q_t+(1-lr)*target_q[transition['action']]
            pred_q=self.evaluateNN(transition['state'])
            loss=tf.losses.mean_squared_error(target_q,pred_q)
            rmse=math.sqrt(sum(pow(target_q-pred_q.numpy()[0],2)))
        grads=t.gradient(loss,self.evaluateNN.trainable_weights)
        self.optimizer.apply_gradients(zip(grads,self.evaluateNN.trainable_weights))
        self.mean_rmse=(self.mean_rmse*self.num_rmse+rmse)/(self.num_rmse+1)
        self.num_rmse+=1

    def getMeanRMSE(self):
        return self.mean_rmse

    def initMeanRMSE(self):
        self.mean_rmse=0
        self.num_rmse=0

    def pred(self,state,e_greedy=0.1):
        q=self.evaluateNN(state).numpy()[0]
        if random.random()>e_greedy:
            action = np.argmax(q)
        else:
            action =np.random.randint(0, len(q))
        return action

    def save(self,path=None):
        if path is None:
            self.evaluateNN.save(self.modelpath)
            print('model saved in'+self.modelpath)
        else:
            self.evaluateNN.save(path)
            print('model saved in '+path)

    def __del__(self):
        if self.autosave:
            self.save()
            print('auto saved')

class ReplayBuffer:
    def __init__(self, capacity=100000):
        self.capacity=capacity
        self.buffer=[]
        self.ptr=0
    def push(self,state,action,reward,nextstate):
        transition={"state":state,"action":action,"reward":reward,"nextstate":nextstate}
        if len(self.buffer)<self.capacity:
            self.buffer.append(transition)
        else:
            self.buffer[self.ptr]=transition
            self.ptr=int((self.ptr+1)%self.capacity)
    def getTransition(self):
        return random.choice(self.buffer)
    
    def __len__(self):
        return len(self.buffer)

class DQNCounter():
    def __init__(self,counter):
        self.num=0
        self.counter=counter

    def isCounter(self):
        iscounter=False
        self.num+=1
        if self.num>self.counter:
            iscounter=True
            self.num=0
        return iscounter

    def Initialize(self):
        self.num=0

def train(dqn,replaybuffer,threshold=1000):
    if len(replaybuffer)>threshold:
        transition=replaybuffer.getTransition()
        dqn.train_step(transition)
    

if __name__=='__main__':
    dqn=DQN(10,[10,10],2,'testmodel',True)
    aa=np.matrix('[1,2,3,4,5,6,7,8,9,10]')
    ab=np.matrix('[1,2,3,4,5,6,7,9,9,10]')
    transition={'state':aa,'action':1,'reward':1,'nextstate':ab}
    dqn.train_step(transition)
    del dqn
    pass