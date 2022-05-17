import time
import tkinter as tk
import random
import math
import numpy as np
import abc
from PIL import Image, ImageTk


class Environment():
    def __init__(self,width=1000,height=1000):
        self.width=width
        self.height=height
        self.window=tk.Tk()
        self.window.resizable(False,False)
        self.canvas=tk.Canvas(self.window,width=width,height=height)
        self.canvas.pack()
        self.deletelist=[]

        self.registry={'active':[],'passive':[],'component':[]}

    def registryEntity(self,entity):
        if not isinstance(entity,EnvironmentEntity):
            assert 'error entity'
        self.registry[entity.getAttribute()].append(entity)
        entity.setEnvironment(self)
        entity.draw(self.canvas)

    def cancelEntity(self,entity):
        if not isinstance(entity,EnvironmentEntity):
            assert 'error entity'
        registry_list=self.registry[entity.getAttribute()]
        self.canvas.delete(entity.getName())
        if entity in registry_list:
            registry_list.remove(entity)
        else:
            pass # warning no entity
    
    def applyCancel(self,entity):
        if not isinstance(entity,EnvironmentEntity):
            assert 'error entity'
        self.deletelist.append(entity)
        

    def moveIt(self,dt=1.0):
        for rr in self.registry['active']:
            rr.move(self.canvas,dt)
        if len(self.deletelist)>0:
            for rr in self.deletelist:
                self.cancelEntity(rr)
            self.deletelist.clear()
        self.canvas.after(50,self.moveIt,dt)

    def getSizeInfo(self):
        return self.width,self.height
    
    def getRegistry(self):
        return self.registry

    def addKey(self,x,y,w,h,text,command):
        button=tk.Button(self.window,text=text,command=command)
        button.place(x=x,y=y,width=w,height=h)

    def strat(self,dt=1.0):
        self.moveIt(dt)
        self.window.mainloop()
        


class EnvironmentEntity(metaclass=abc.ABCMeta):
    def __init__(self,name,x,y,theta,attribute):
        self.name=name
        self.x=x
        self.y=y
        self.theta=theta
        self.attribute=attribute
        self.env=None

    def getAttribute(self):
        return self.attribute

    def getLocation(self):
        return self.x, self.y
    
    def getTheta(self):
        return self.theta

    def getName(self):
        return self.name
    
    def boundaryCheck(self):
        w,h=self.env.getSizeInfo()
        if self.x>w:
            self.x=w-1
        if self.x<0:
            self.x=1
        if self.y>h:
            self.y=h-1
        if self.y<0:
            self.y=1
    def cyclicBoundary(self):
        w,h=self.env.getSizeInfo()
        if self.x>w:
            self.x=1
        if self.x<0:
            self.x=w-1
        if self.y>h:
            self.y=1
        if self.y<0:
            self.y=h-1

    def distanceTo(self,entity):
        if not isinstance(entity,EnvironmentEntity):
            assert "error entity"
        x,y=entity.getLocation()
        distance=math.sqrt(math.pow(self.x-x,2)+math.pow(self.y-y,2))
        return distance
    
    def setEnvironment(self,env):
        if not isinstance(env,Environment):
            assert 'error environment'
        self.env=env

    @abc.abstractclassmethod
    def draw(self,canvas):pass


class MainEntity(EnvironmentEntity,metaclass=abc.ABCMeta):
    def __init__(self, name, x, y,theta,attribute):
        super().__init__(name, x, y, theta,attribute)
        self.component=[]
    def registryComponet(self,component):
        if not isinstance(component,ComponentEntity):
            assert "error ComponentEntity"
        self.component.append(component)
    def getComponet(self):
        return self.component

class ActiveEntity(MainEntity,metaclass=abc.ABCMeta):
    attribute='active'
    def __init__(self, name, x, y,theta):
        super().__init__(name, x, y,theta,'active')
    
    def move(self,canvas,dt):
        self.transferFunction()

    def transferFunction(self):pass

class PassiveEntity(MainEntity,metaclass=abc.ABCMeta):
    attribute='passive'
    def __init__(self, name, x, y,theta):
        super().__init__(name, x, y, theta, 'passive')

class ComponentEntity(EnvironmentEntity,metaclass=abc.ABCMeta):
    attribute='component'
    def __init__(self, name, entity,x=0,y=0,theta=0):
        super().__init__(name, None, None, None,'component')
        if not isinstance(entity,MainEntity):
            assert 'error mainentity'
        self.main_entity=entity
        self.main_entity.registryComponet(self)
        self.relative_x=x
        self.relative_y=y
        self.relative_theta=theta
        main_entity_x,main_entity_y=entity.getLocation()
        main_theta=entity.getTheta()
        self.theta=main_theta+self.relative_theta
        self.x=main_entity_x+self.relative_x*math.cos(-main_theta)+self.relative_y*math.sin(-main_theta)
        self.y=main_entity_y+self.relative_y*math.cos(-main_theta)-self.relative_x*math.sin(-main_theta)
        self.name=self.main_entity.getName()+"_"+self.name

    def getRelativeLocation(self):
        return self.relative_x,self.relative_y

    def getRelativeTheta(self):
        return self.relative_theta

    def getMainEntity(self):
        return self.main_entity

    def changeFunction(self):
        pass

    def move(self,canvas):
        main_entity_x,main_entity_y=self.main_entity.getLocation()
        main_theta=self.main_entity.getTheta()
        self.x=main_entity_x+self.relative_x
        self.y=main_entity_y+self.relative_y
        self.theta=main_theta+self.relative_theta
        self.x=main_entity_x+self.relative_x*math.cos(-main_theta)+self.relative_y*math.sin(-main_theta)
        self.y=main_entity_y+self.relative_y*math.cos(-main_theta)-self.relative_x*math.sin(-main_theta)
        self.changeFunction()
        canvas.delete(self.name)
        self.draw(canvas)




# bot
class Bot(ActiveEntity):
    def __init__(self,name,x,y,theta):
        super(Bot,self).__init__(name,x,y,theta)
        # self.theta = random.uniform(0.0,2.0*math.pi)

        self.ll = 60 #axle width 与车的指标有关-宽度
        self.vl = 0.0 #左轮速度
        self.vr = 0.0 #右轮速度

        self.battery=None
        self.sensorchargerL=None
        self.sensorchargerR=None

        self.dirtcollected=0
        self.numcollision=0
        self.num_collision_boundary=0
        
    def registryComponet(self, component):
        super().registryComponet(component)
        if isinstance(component,Battery):
            self.battery=component
        if component.getName() =="sensorchargerL":
            self.sensorchargerL=component
        if component.getName() =="sensorchargerR":
            self.sensorchargerR=component

    def setEnvironment(self, env):
        super().setEnvironment(env)

    # 在环境中创建/画车体
    def draw(self,canvas):
        # 长60宽60的车体
        points = [ (self.x + 30*math.sin(self.theta)) - 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y - 30*math.cos(self.theta)) - 30*math.cos((math.pi/2.0)-self.theta), \
                   (self.x - 30*math.sin(self.theta)) - 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y + 30*math.cos(self.theta)) - 30*math.cos((math.pi/2.0)-self.theta), \
                   (self.x - 30*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y + 30*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta), \
                   (self.x + 30*math.sin(self.theta)) + 30*math.sin((math.pi/2.0)-self.theta), \
                   (self.y - 30*math.cos(self.theta)) + 30*math.cos((math.pi/2.0)-self.theta)  \
                ]
        canvas.create_polygon(points, fill="blue", tags=self.name)
        
        # 车轮
        wheel1PosX = self.x - 30*math.sin(self.theta)
        wheel1PosY = self.y + 30*math.cos(self.theta)
        canvas.create_oval(wheel1PosX-3,wheel1PosY-3,\
                                         wheel1PosX+3,wheel1PosY+3,\
                                         fill="red",tags=self.name)

        wheel2PosX = self.x + 30*math.sin(self.theta)
        wheel2PosY = self.y - 30*math.cos(self.theta)
        canvas.create_oval(wheel2PosX-3,wheel2PosY-3,\
                                         wheel2PosX+3,wheel2PosY+3,\
                                         fill="green",tags=self.name)

    # 实现整个车体在环境中的移动和传感器问题
    def move(self,canvas,dt):
        super().move(canvas,dt)

        if not self.battery is None:
            if self.battery.getEnergy()==0:
                self.vl=0
                self.vr=0

        if self.vl==self.vr:
            R = 0
        else:
            R = (self.ll/2.0)*((self.vr+self.vl)/(self.vl-self.vr))
        omega = (self.vl-self.vr)/self.ll
        ICCx = self.x-R*math.sin(self.theta) #instantaneous centre of curvature
        ICCy = self.y+R*math.cos(self.theta)
        m = np.matrix( [ [math.cos(omega*dt), -math.sin(omega*dt), 0], \
                        [math.sin(omega*dt), math.cos(omega*dt), 0],  \
                        [0,0,1] ] )
        v1 = np.matrix([[self.x-ICCx],[self.y-ICCy],[self.theta]])
        v2 = np.matrix([[ICCx],[ICCy],[omega*dt]])
        newv = np.add(np.dot(m,v1),v2)
        newX = newv.item(0)
        newY = newv.item(1)
        newTheta = newv.item(2)
        newTheta = newTheta%(2.0*math.pi) #make sure angle doesn't go outside [0.0,2*pi)
        self.x = newX
        self.y = newY
        self.theta = newTheta        
        if self.vl==self.vr: # straight line movement
            self.x += self.vr*math.cos(self.theta) #vr wlog
            self.y += self.vr*math.sin(self.theta)     
        # 边界问题
        self.cyclicBoundary()
        # self.boundaryCheck()
        canvas.delete(self.name)
        self.draw(canvas)
        # component move
        for rr in self.component:
            rr.move(canvas)
        # dirt
        self.collectDirt()
        # collision
        self.collision(canvas)

    def collectDirt(self):
        registryPassives=self.env.getRegistry().get('passive')
        for rr in registryPassives:
            if isinstance(rr,Dirt):
                if self.distanceTo(rr)<36:
                    self.env.applyCancel(rr)
                    self.dirtcollected+=1

    def collision(self,canvas):
        for rr in self.env.getRegistry().get('active'):
            if isinstance(rr,Cat):
                if self.distanceTo(rr)<50.0:
                    self.numcollision+=1
                    self.env.applyCancel(rr)

    def collision_boundary(self):
        w,h=self.env.getSizeInfo()
        if self.x<1 or self.x>w-1 or self.y<1 or self.y>h-1:
            self.num_collision_boundary+=1


    def getDirtCollected(self):
        return self.dirtcollected

    def getcollision(self):
        return self.numcollision

    # 控制轮胎速度的接口
    def transferFunction(self):
        # wheels not moving - no movement
        # self.vl = 2.0
        # self.vr = -2.0
        pass


class Cat(ActiveEntity):
    def __init__(self, name, x, y, theta):
        super().__init__(name, x, y, theta)
        self.vl = 1.0
        self.vr = 1.0
        self.turning = 0
        self.moving = random.randrange(50,100)
        self.currentlyTurning = False
        self.ll = 20 #axle width
        imgFile = Image.open("cat.png")
        imgFile = imgFile.resize((30,30), Image.ANTIALIAS)
        self.image = ImageTk.PhotoImage(imgFile)
    

    def setEnvironment(self, env):
        super().setEnvironment(env)

    def draw(self,canvas):
        body = canvas.create_image(self.x,self.y,image=self.image,tags=self.name)

    def move(self,canvas,dt):
        super().move(canvas,dt)
        if self.vl==self.vr:
            R = 0
        else:
            R = (self.ll/2.0)*((self.vr+self.vl)/(self.vl-self.vr))
        omega = (self.vl-self.vr)/self.ll
        ICCx = self.x-R*math.sin(self.theta) #instantaneous centre of curvature
        ICCy = self.y+R*math.cos(self.theta)
        m = np.matrix( [ [math.cos(omega*dt), -math.sin(omega*dt), 0], \
                        [math.sin(omega*dt), math.cos(omega*dt), 0],  \
                        [0,0,1] ] )
        v1 = np.matrix([[self.x-ICCx],[self.y-ICCy],[self.theta]])
        v2 = np.matrix([[ICCx],[ICCy],[omega*dt]])
        newv = np.add(np.dot(m,v1),v2)
        newX = newv.item(0)
        newY = newv.item(1)
        newTheta = newv.item(2)
        newTheta = newTheta%(2.0*math.pi) #make sure angle doesn't go outside [0.0,2*pi)
        self.x = newX
        self.y = newY
        self.theta = newTheta        
        if self.vl==self.vr: # straight line movement
            self.x += self.vr*math.cos(self.theta) #vr wlog
            self.y += self.vr*math.sin(self.theta)
        self.cyclicBoundary()
        # self.boundaryCheck()
        #self.updateMap()
        canvas.delete(self.name)
        self.draw(canvas)

    def jump(self,canvas,r_min=50,r_max=100):
        self.x+=random.randint(r_min,r_max)
        self.y+=random.randint(r_min,r_max)
        self.cyclicBoundary()
        canvas.delete(self.name)
        self.draw(canvas)

    def transferFunction(self):
        super().transferFunction()
        if self.currentlyTurning==True:
            self.vl = -2.0
            self.vr = 2.0
            self.turning -= 1
        else:
            self.vl = 1.0
            self.vr = 1.0
            self.moving -= 1
        if self.moving==0 and not self.currentlyTurning:
            self.turning = random.randrange(20,40)
            self.currentlyTurning = True
        if self.turning==0 and self.currentlyTurning:
            self.moving = random.randrange(50,100)
            self.currentlyTurning = False

class Scoreboard(ActiveEntity):
    def __init__(self, name, x, y,bot):
        super().__init__(name, x, y, 0)
        self.bot=bot
        self.initialize()

    def draw(self,canvas):
        canvas.create_text(self.x,self.y,\
                            text="Time: "+str(self.time)+\
                                "\nDirt collected: "+str(self.dirtcollected)+\
                                "\ncollision: "+str(self.numcollision)+\
                                "\nScore: "+str(self.score),tags=self.name)
    def initialize(self):
        self.starttime=time.time()
        self.dirtcollected=0
        self.time=0
        self.score=0
        self.numcollision=0
        self.reward=0

        self.botenergy=self.bot.battery.getEnergy()
        self.isenergyzero=False
        self.energythreshold=500
        self.underthreshold=False
        self.minsensorchager=math.inf
        self.last_min_nearcat=math.inf

    def rewardAboutToward(self):
        bot_x,bot_y=self.bot.getLocation()
        bot_theta=self.bot.getTheta()
        registryactive=self.env.getRegistry()['active']
        reward=0
        istoward=False
        for rr in registryactive:
            if isinstance(rr,Cat):
                dis=rr.distanceTo(self.bot)
                if dis<90:
                    cat_x,cat_y=rr.getLocation()
                    cat_x=cat_x-bot_x
                    cat_y=cat_y-bot_y
                    cat_rx=cat_x*math.cos(bot_theta)+cat_y*math.sin(bot_theta)
                    cat_ry=-cat_x*math.sin(bot_theta)+cat_y*math.cos(bot_theta)
                    if cat_rx>30 and cat_ry<50 and cat_ry>-50:
                        istoward=True
        if istoward:
            reward=-0.1
        return reward

    def rewardAboutNear(self):
        reward=0
        registryactive=self.env.getRegistry()['active']
        isnear=False
        for rr in registryactive:
            if isinstance(rr,Cat):
                dis=rr.distanceTo(self.bot)
                if dis<90:
                    isnear=True
                    if dis<self.last_min_nearcat:
                        reward-=(90-dis)/90
                        self.last_min_nearcat=dis
        if not isnear:
            self.last_min_nearcat=math.inf
        return reward
    
    def move(self, canvas, dt):
        super().move(canvas, dt)
        self.time=round(time.time()-self.starttime,1)
        reward=0
        if self.bot.getDirtCollected()>self.dirtcollected:
            addcollected=self.bot.getDirtCollected()-self.dirtcollected
            reward+=addcollected*1
            self.dirtcollected=self.bot.getDirtCollected()
        if self.bot.getcollision()>self.numcollision:
            addcollision=self.bot.getcollision()-self.numcollision
            reward-=addcollision*3
            self.numcollision=self.bot.getcollision()

        # The more closer,the more punishment
        reward+=self.rewardAboutNear()
                
        # in 90,Toward the cat -0.1
        # reward+=self.rewardAboutToward()

        nowbotenergy=self.bot.battery.getEnergy()
        nowsensorcharger=self.bot.sensorchargerL.sensor()+self.bot.sensorchargerR.sensor()
        # Less than the threshold - near Changer
        if nowbotenergy<self.energythreshold:
            self.underthreshold=True
            if self.botenergy<nowbotenergy:
                # reward+=1
                self.minsensorchager=math.inf
                pass
            else:
                if nowsensorcharger<self.minsensorchager:
                    self.minsensorchager=nowsensorcharger
                    # reward+=1

        # zero energy -20
        if nowbotenergy==0 and not self.isenergyzero:
            # reward-=20
            self.isenergyzero=True
        #  less than the threshold previously but now full +10
        if self.underthreshold and nowbotenergy==self.bot.battery.max_energy:
            self.underthreshold=False
            # reward+=10
        if self.isenergyzero:
            if nowbotenergy!=0:
                self.isenergyzero=False
        self.botenergy=nowbotenergy
        self.reward=reward
        self.score+=reward


        canvas.itemconfigure(self.name,\
                            text="Time: "+str(self.time)+" s"+\
                                "\nDirt collected: "+str(self.dirtcollected)+\
                                "\ncollision: "+str(self.numcollision)+\
                                "\nScore: "+str(int(self.score)))

    def getScore(self):
        return self.score

    def getReward(self):
        return self.reward


class Battery(ComponentEntity):
    def __init__(self, name, entity, energy=1000,rate=1,x=0, y=0, theta=0):
        super().__init__(name, entity, x, y, theta)
        self.max_energy=energy
        self.rate=rate
        self.energy=energy

    def setEnergy(self,energy):
        self.energy=energy
    
    def getEnergy(self):
        return self.energy

    def sensor(self):
        return self.energy/self.max_energy

    def draw(self,canvas):
        centre1PosX = self.x 
        centre1PosY = self.y
        canvas.create_oval(centre1PosX-15,centre1PosY-15,\
                           centre1PosX+15,centre1PosY+15,\
                           fill="gold",tags=self.name)
        canvas.create_text(self.x,self.y,text=str(self.energy),tags=self.name)

    def consume(self):
        if self.energy>0:
            self.energy-=self.rate
        if self.energy<0:
            self.energy=0
    
    def charge(self):
        registryPassives=self.env.getRegistry().get('passive')
        for rr in registryPassives:
            if isinstance(rr,Charger) and self.distanceTo(rr)<=rr.getValidDistance():
                self.energy+=rr.charge()
                if self.energy>self.max_energy:
                    self.energy=self.max_energy

    def changeFunction(self):
        self.consume()
        self.charge()

class SensorLaser(ComponentEntity):
    def __init__(self, name, entity, x=0, y=0, theta=0,target_entiey=None,vaild_distance=400):
        super().__init__(name, entity, x, y, theta)
        self.vaild_dis=vaild_distance
        self.target_entiey=target_entiey
    
    def draw(self,canvas):
        canvas.create_oval(self.x-2,self.y-2,\
                            self.x+2,self.y+2,\
                            fill="purple1",tags=self.name)
        canvas.create_line(self.x,self.y,\
                            self.x+self.vaild_dis*math.cos(self.theta),self.y+self.vaild_dis*math.sin(self.theta),\
                            fill="light grey",tags=self.name)

    def sensor(self):
        sensor_data=0
        registryActives=self.env.getRegistry().get(self.target_entiey.attribute)
        for rr in registryActives:
            if isinstance(rr,self.target_entiey):
                dd=self.distanceTo(rr)
                scaledDistance = max(self.vaild_dis-dd,0)/self.vaild_dis
                if scaledDistance>0:
                    ncx = rr.x-self.x #cat if robot were at 0,0
                    ncy = rr.y-self.y
                    r_ncx=ncx*math.cos(self.theta)+ncy*math.sin(self.theta)
                    if r_ncx>0:
                        m = math.tan(self.theta)
                        A = m*m+1
                        B = 2*(-m*ncy-ncx)
                        r = 15 #radius
                        C = ncy*ncy - r*r + ncx*ncx
                        if B*B-4*A*C>=0:
                            sensor_data=scaledDistance
        return sensor_data

class SensorRadar(ComponentEntity):
    def __init__(self, name, entity, x=0, y=0, theta=0,target_entiey=None,vaild_distance=400,resolution=360):
        super().__init__(name, entity, x, y, theta)
        self.vaild_dis=vaild_distance
        self.resolution=resolution
        self.target_entiey=target_entiey

    def draw(self,canvas):
            canvas.create_oval(self.x-self.vaild_dis,self.y-self.vaild_dis,self.x+self.vaild_dis,self.y+self.vaild_dis,outline="black",tags=self.name)
    
    def sensor(self):
        sensor_data=[0]*self.resolution
        registryActives=self.env.getRegistry().get(self.target_entiey.attribute)
        for rr in registryActives:
            if isinstance(rr,self.target_entiey):
                dd=self.distanceTo(rr)
                if dd<=self.vaild_dis:
                    scaledDistance = max(self.vaild_dis-dd,0)/self.vaild_dis
                    ncx = rr.x-self.x #cat if robot were at 0,0
                    ncy = rr.y-self.y
                    r_ncx=ncx*math.cos(self.theta)+ncy*math.sin(self.theta)
                    r_ncy=-ncx*math.sin(self.theta)+ncy*math.cos(self.theta)
                    if r_ncx==0:
                        if r_ncy>0:
                            radian=math.pi/2
                        else:
                            radian=3*math.pi/2
                    else:
                        radian=math.atan(r_ncy/r_ncx)
                    if r_ncx<0 and r_ncy>0:
                        radian=math.pi+radian
                    if r_ncx<0 and r_ncy<0:
                        radian=math.pi+radian
                    if r_ncx>0 and r_ncy<0:
                        radian=2*math.pi+radian
                    ptr=int(self.resolution*radian/(2*math.pi))
                    if scaledDistance>sensor_data[ptr]:
                            sensor_data[ptr]=scaledDistance
                    rg=self.resolution/8
                    num=int(rg*scaledDistance/2)
                    for i in range(num):
                        p=ptr+i+1
                        if p>=len(sensor_data):
                            p=p-len(sensor_data)
                        if scaledDistance>sensor_data[p]:
                            sensor_data[p]=scaledDistance
                        if scaledDistance>sensor_data[ptr-i-1]:
                            sensor_data[ptr-i-1]=scaledDistance
                        
        return sensor_data

class ViewRadar(ActiveEntity):
    def __init__(self, name, x, y):
        super().__init__(name, x, y, 0)
        self.sensorradar=None

    def draw(self,canvas):
        sensordata=self.sensorradar.sensor()
        for i in range(len(sensordata)):
            if sensordata[i]==0:
                canvas.create_rectangle(self.x+i*2,self.y,self.x+i*2+2,self.y+15,fill="white",tags=self.name)
            else:
                colour = hex(15-math.floor(sensordata[i]*16.0)) #scale to 0-15 -> hex
                fillHex = "#"+colour[2]+colour[2]+colour[2]
                canvas.create_rectangle(self.x+i*2,self.y,self.x+i*2+2,self.y+15,fill=fillHex,tags=self.name)
    def move(self, canvas, dt):
        super().move(canvas, dt)
        canvas.delete(self.name)
        self.draw(canvas)

    def bindSensorRadar(self,sensorradar):
        if not isinstance(sensorradar,SensorRadar):
            assert 'error SensorRadar'
        self.sensorradar=sensorradar

class ViewLaser(ActiveEntity):
    def __init__(self, name, x, y):
        super().__init__(name, x, y, 0)
        self.sensorlaser=[]

    def draw(self,canvas):
        for vv in range(len(self.sensorlaser)):
            if self.sensorlaser[vv].sensor()==0:
                canvas.create_rectangle(self.x+vv*15,self.y,self.x+vv*15+15,self.y+15,fill="white",tags=self.name)
            if self.sensorlaser[vv].sensor()>0:
                colour = hex(15-math.floor(self.sensorlaser[vv].sensor()*16.0)) #scale to 0-15 -> hex
                fillHex = "#"+colour[2]+colour[2]+colour[2]
                canvas.create_rectangle(self.x+vv*15,self.y,self.x+vv*15+15,self.y+15,fill=fillHex,tags=self.name)

    def move(self, canvas, dt):
        super().move(canvas, dt)
        canvas.delete(self.name)
        self.draw(canvas)

    def bindSensorLaser(self,sensorlaser):
        if not isinstance(sensorlaser,SensorLaser):
            assert 'error sensorlaser'
        self.sensorlaser.append(sensorlaser)


class SensorCharger(ComponentEntity):
    def __init__(self, name, entity, x=0, y=0):
        super().__init__(name, entity, x, y, 0)

    def draw(self,canvas):
        canvas.create_oval(self.x-3,self.y-3, \
                           self.x+3,self.y+3, \
                           fill="yellow",tags=self.name)

    def sensor(self):
        sensor_data=0
        registryPassives=self.env.getRegistry().get('passive')
        for rr in registryPassives:
            if isinstance(rr,Charger):
                distance=self.distanceTo(rr)
                sensor_data+=distance/self.env.width
        return sensor_data




class Charger(PassiveEntity):
    def __init__(self, name, x, y,rate=10,dis=80):
        super().__init__(name, x, y,0)
        self.rate=rate
        self.valid_distance=dis
        
    def draw(self,canvas):
        body = canvas.create_oval(self.x-10,self.y-10, \
                                  self.x+10,self.y+10, \
                                  fill="gold",tags=self.name)
    def charge(self):
        return self.rate
    
    def getValidDistance(self):
        return self.valid_distance

class Dirt(PassiveEntity):
    def __init__(self, name,x,y):
        super().__init__(name, x, y, 0)

    def draw(self,canvas):
        body = canvas.create_oval(self.x-1,self.y-1, \
                                  self.x+1,self.y+1, \
                                  fill="grey",tags=self.name)

if __name__=='__main__':
    width=1000
    height=800
    env=Environment(width,height)
    bot=Bot('bot1',100,100,math.pi/2)

    env.registryEntity(bot)
    env.registryEntity(Battery("battery1",bot))
    env.registryEntity(SensorCharger("sensorchargerL",bot,30,-20))
    env.registryEntity(SensorCharger("sensorchargerR",bot,30,20))
    env.registryEntity(Charger('charger1',500,500))
    sensorradar=SensorRadar("radar1",bot,0,0,-math.pi,Cat,400,360)
    viewradar=ViewRadar('viewradar',140,40)
    viewradar.bindSensorRadar(sensorradar)
    env.registryEntity(sensorradar)
    env.registryEntity(viewradar)
    viewbar_front=ViewLaser("viewbar_front",850,70)
    env.registryEntity(viewbar_front)
    for pos in range(-20,21,5):
        sensorlaser=SensorLaser("laser_front"+str(pos),bot,30,pos,0,Dirt,100)
        env.registryEntity(sensorlaser)
        viewbar_front.bindSensorLaser(sensorlaser)
    num_act=10
    for i in range(0,num_act):
        env.registryEntity(Cat('cat'+str(i),random.randint(100,0.9*width),random.randint(100,0.9*height),random.uniform(0.0,2.0*math.pi)))
    num_dirt=300
    for i in range(0,num_dirt):
        env.registryEntity(Dirt('dirt'+str(i),random.randint(100,0.9*width),random.randint(100,0.9*height)))
    
    scoreboard=Scoreboard('scoreboard',70,50,bot)
    env.registryEntity(scoreboard)
    scoreboard.initialize()
    env.strat()
