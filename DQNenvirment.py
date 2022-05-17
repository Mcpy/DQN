from environment import *
from DQN import *



class TransitionBuffer(ActiveEntity):
    def __init__(self, replaybuffer,maxcounter=4,frames=3):
        super().__init__('transitionbuffer', 0, 0, 0)
        self.replaybuffer=replaybuffer
        self.interval=maxcounter
        self.intervalcounter=0
        self.framescounter=DQNCounter(frames)
        self.frames_next_counter=DQNCounter(frames)
        self.isnextstate=False
        self.transition={'state':None,'action':None,'reward':None}
        self.rewardinfo=None
        self.DQNbot=None

        self.frames_state=[]
        self.frames_raward=0
        self.frames_action=0
        self.frames_nextstate=[]
        
    
    def BindDQNBot(self,DQNbot):
        self.DQNbot=DQNbot

    def bindRewardinfo(self,rewardinfo):
        self.rewardinfo=rewardinfo

    def getState(self):
        state=[]
        for rr in self.DQNbot.statrsensor:
            try:
                sensor_data=rr.sensor()
            except:
                continue
            if isinstance(sensor_data,list):
                state.extend(sensor_data)
            else:
                state.append(sensor_data)
        return state

    def draw(self,canvas):pass


    def move(self, canvas, dt):
        super().move(canvas, dt)
        self.intervalcounter+=1
        if self.isnextstate:
            if not self.frames_next_counter.isCounter():
                self.frames_nextstate.extend(self.getState())
            else:
                self.replaybuffer.push(self.transition['state'],self.transition['action'],\
                                        self.transition['reward'],np.matrix(self.frames_nextstate))
                self.frames_nextstate=[]
                self.isnextstate=False
        if self.intervalcounter>self.interval:
            if not self.framescounter.isCounter():
                if self.framescounter.num ==1:
                    self.frames_action=self.DQNbot.getAction()
                self.frames_state.extend(self.getState())
                self.frames_raward+=self.rewardinfo.getReward()
            else:
                self.transition['state']=np.matrix(self.frames_state)
                self.frames_state=[]
                self.transition['action']=self.frames_action
                self.transition['reward']=self.frames_raward
                self.frames_raward=0
                self.isnextstate=True
                self.intervalcounter=0

class DQNBot(Bot):
    def __init__(self, name, x, y, theta,dqn,action_interval=3,frames=3):
        super().__init__(name, x, y, theta)
        self.DQN=dqn
        self.actionlist=[self.forward,self.lowForward,self.turnRight,self.turnLeft]
        self.action=0
        self.counter=DQNCounter(action_interval)
        self.statrsensor=[]

        self.frames_state=[]
        self.frames=frames
    
    def registryComponet(self, component):
        super().registryComponet(component)
        if isinstance(component,SensorLaser) or isinstance(component,SensorRadar):
            self.statrsensor.append(component) 

    def forward(self):
        self.vl=5.0
        self.vr=5.0
    
    def lowForward(self):
        self.vl=2.5
        self.vr=2.5

    def back(self):
        self.vl=-2
        self.vr=-2

    def turnRight(self):
        self.vl=2
        self.vr=-2

    def turnLeft(self):
        self.vl=-2
        self.vr=2

    def stop(self):
        self.vl=0
        self.vr=0

    def getState(self):
        state=[]
        for rr in self.statrsensor:
            try:
                sensor_data=rr.sensor()
            except:
                continue
            if isinstance(sensor_data,list):
                state.extend(sensor_data)
            else:
                state.append(sensor_data)
        return state

    def getAction(self):
        return self.action

    def transferFunction(self):
        super().transferFunction()
        self.frames_state.append(self.getState())
        if self.frames < len(self.frames_state):
            self.frames_state.pop(0)
        if self.counter.isCounter():
            if len(self.frames_state)==self.frames:
                state=[]
                for i in self.frames_state:
                    state.extend(i)
                self.action=self.DQN.pred(np.matrix(state))
        self.actionlist[self.action]()


class EnvironmentController(ActiveEntity):
    def __init__(self,bot,scoreboard,dirtnum=200,catnum=10,recreate_time_s=60):
        super().__init__('EnvironmentController', 0, 0, 0)
        self.bot=bot
        self.scoreboard=scoreboard
        self.dirtnum=dirtnum
        self.adddirtnum=0
        self.catnum=catnum
        self.addcatnum=0
        self.time=EnvirmentTimer(recreate_time_s)
    def draw(self,canvas):pass

    def move(self, canvas, dt):
        super().move(canvas, dt)
        # getEnergy=0 game over and reset
        if self.bot.battery.getEnergy()==0 or self.time.isClock():
            print('collected: '+str(self.bot.dirtcollected)+' collision: '+str(self.bot.numcollision))
            self.bot.battery.setEnergy(self.bot.battery.max_energy)
            self.bot.numcollision=0
            self.bot.dirtcollected=0
            self.scoreboard.initialize()
            self.time.initialize()
            self.entityCreater()

    def entityCreater(self):
        registrypassive=self.env.getRegistry()['passive']
        numdirt=0
        for rr in registrypassive:
            if isinstance(rr,Dirt):
                numdirt+=1
        if numdirt<self.dirtnum:
            for i in range(0,self.dirtnum-numdirt):
                self.adddirtnum+=1
                self.env.registryEntity(Dirt('dirt_add'+str(self.adddirtnum),random.randint(100,0.9*self.env.width),random.randint(100,0.9*self.env.height)))
        registryactive=self.env.getRegistry()['active']
        numcat=0
        for rr in registryactive:
            if isinstance(rr,Cat):
                numcat+=1
        if numcat<self.catnum:
            for i in range(0,self.catnum-numcat):
                self.addcatnum+=1
                self.env.registryEntity(Cat('cat_add'+str(self.addcatnum),random.randint(100,0.9*self.env.width),random.randint(100,0.9*self.env.height),random.uniform(0.0,2.0*math.pi)))
            

class DQNTrain(ActiveEntity):
    def __init__(self,dqn,replaybuffer,threshold=1000,copycounter=100):
        super().__init__('DQNTrain', 0, 0, 0)
        self.dqn=dqn
        self.replaybuffer=replaybuffer
        self.threshold=threshold
        self.isstart=False
        self.counter=DQNCounter(copycounter)

    def draw(self,canvas):pass

    def move(self, canvas, dt):
        super().move(canvas, dt)
        if len(self.replaybuffer)>self.threshold:
            if not self.isstart:
                print('start train')
                self.isstart=True
            transition=self.replaybuffer.getTransition()
            self.dqn.train_step(transition,0.9)
            if self.counter.isCounter():
                self.dqn.copyNN()
                print('Mean RMSE: '+ str(self.dqn.getMeanRMSE()))
                self.dqn.initMeanRMSE()


class ModelSaver(ActiveEntity):
    def __init__(self,counter,dqn,path='model/'):
        super().__init__('modelsaver', 0, 0, 0)
        self.counter=DQNCounter(counter)
        self.dqn=dqn
        self.num=0
        self.path=path

    def draw(self,canvas):pass

    def move(self, canvas, dt):
        super().move(canvas, dt)
        if self.counter.isCounter():
            self.num+=1
            self.dqn.save(self.path+str(self.num))

class EnvirmentTimer():
    def __init__(self,time_s):
        self.time=time.time()
        self.t_s=time_s

    def isClock(self):
        isclock=False
        nowtime=time.time()
        if nowtime-self.time>=self.t_s:
            self.time=nowtime
            isclock=True
        return isclock

    def initialize(self):
        self.time=time.time()




