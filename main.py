from environment import *
from DQN import *
from DQNenvirment import *

# dqn
state_frames=3
radar_resolution=72
laser_num=9
sensor_state=radar_resolution+laser_num
replaybuffer=ReplayBuffer(100000)
dqn=DQN(state_frames*sensor_state,[486,256],4,lr=1e-6)

# Environment
width=1000
height=800
env=Environment(width,height)
env.addKey(880,80,75,30,'save model',dqn.save)

# entity

# bot and its component
bot=DQNBot('bot1',100,100,math.pi/2,dqn,action_interval=1,frames=state_frames)
env.registryEntity(bot)
env.registryEntity(Battery("battery1",bot))
env.registryEntity(SensorCharger("sensorchargerL",bot,30,-20))
env.registryEntity(SensorCharger("sensorchargerR",bot,30,20))

# set Charger
env.registryEntity(Charger('charger1',500,500))

# set cat
num_act=10
for i in range(0,num_act):
    env.registryEntity(Cat('cat'+str(i),random.randint(100,0.9*width),random.randint(100,0.9*height),random.uniform(0.0,2.0*math.pi)))

# set dirt
num_dirt=100
for i in range(0,num_dirt):
    env.registryEntity(Dirt('dirt'+str(i),random.randint(100,0.9*width),random.randint(100,0.9*height)))

# bot's Radar and view
sensorradar=SensorRadar("radar1",bot,0,0,-math.pi,Cat,vaild_distance= 400,resolution= 72)
env.registryEntity(sensorradar)
viewradar=ViewRadar('viewradar',140,40)
viewradar.bindSensorRadar(sensorradar)
env.registryEntity(viewradar)
# bot's Laser and view
viewbar_front=ViewLaser("viewbar_front",850,60)
for pos in range(-20,21,5):
    sensorlaser=SensorLaser("laser_front"+str(pos),bot,30,pos,0,Dirt,100)
    env.registryEntity(sensorlaser)
    viewbar_front.bindSensorLaser(sensorlaser)
env.registryEntity(viewbar_front)

# set Scoreboard
scoreboard=Scoreboard('scoreboard',70,50,bot)
env.registryEntity(scoreboard)

# set botbuffer
botbuffer=TransitionBuffer(replaybuffer,maxcounter= 0,frames= state_frames)
botbuffer.bindRewardinfo(scoreboard)
botbuffer.BindDQNBot(bot)
env.registryEntity(botbuffer)


# set EnvironmentController
env.registryEntity(EnvironmentController(bot,scoreboard,dirtnum=30,catnum=10,recreate_time_s=120))

# DQNTrain
env.registryEntity(DQNTrain(dqn,replaybuffer,threshold=1000,copycounter=100))

# modelsaverd
env.registryEntity(ModelSaver(10000,dqn,'model/botmodel_'))

# start
scoreboard.initialize()
env.strat()