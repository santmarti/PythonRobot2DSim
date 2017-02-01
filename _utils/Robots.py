import numpy as np 
import Box2D
from Box2DWorld import (world, step, createBox, createBoxFixture, createCircle, 
                        myCreateRevoluteJoint, vrotate, RayCastCallback)
from Arm import Arm
from VectorFigUtils import dist


class IR:
    def __init__(self, nir=1):
        self.nir = nir
        self.maxdist = 3
        self.callback = RayCastCallback()
        if(nir < 4): m, da = (1+nir)%2, np.pi/(2+nir)
        else: m, da = (1+nir)%2, np.pi/(nir-1)
        self.IRAngles = [k*da-((nir-m)/2)*da-m*da/2 for k in range(nir)]
        self.IRValues = [1 for i in range(nir)]

    def update(self, pos, angle, r=0.1):
        for k in range(self.nir):
            v = vrotate((1,0), angle + self.IRAngles[k]) 
            c = pos+[0.9*r*v[0],0.9*r*v[1]]
            cdist = pos+[self.maxdist*v[0],self.maxdist*v[1]]            
            self.callback.fixture = None
            world.RayCast(self.callback, c, cdist)
            if(self.callback.fixture != None):
                self.IRValues[k] = dist(c,self.callback.point) / self.maxdist
            else:
                self.IRValues[k] = 1

# *****************************************************************
# Simple Robot class with two arms of any number of joints

class Epuck:

    def __init__(self, position=(0,0), r=0.48, bHorizontal = False, bMatplotlib=False, frontIR = 6):
        self.ini_pos = position
        x,y = position[0], position[1]
        self.body = createCircle(position, r=r, bDynamic=True, bMatplotlib=True, name="epuck")
        self.body.angle = np.pi/2
        self.r = r
        #self.body = createBox(position, w=0.2,h=0.2,bDynamic=True, bMatplotlib=True)
        self.motors = [0,0]
        self.bHorizontal = bHorizontal

        self.frontIR = frontIR
        self.IR = IR(frontIR)
        self.body.userData["nIR"]=frontIR
        self.body.userData["IRAngles"] = self.IR.IRAngles
        self.body.userData["IRValues"] = self.IR.IRValues
        self.body.userData["reward"]=0
        

    def getIRs(self):
        return self.body.userData["IRValues"]

    def stop(self):
        self.body.linearVelocity = [0,0]
        self.body.angularVelocity = 0
        step()

    def update(self):
        body,angle,pos = self.body, self.body.angle,self.body.position
        mLeft,mRight =  self.motors
        fangle,fdist = 50*(mRight - mLeft), 1000*(mLeft + mRight)
        d = (fdist*np.cos(angle), fdist*np.sin(angle)) 
        #self.body.linearVelocity = f

        if(self.bHorizontal): d = vrotate(d,np.pi/2) 

        self.body.ApplyTorque(fangle,wake=True)
        self.body.ApplyForce(force=d,point=self.body.worldCenter,wake=False)

        if(self.bHorizontal):
            self.body.angularVelocity = 0
            self.body.angle = np.pi / 2

        nir = self.frontIR 
        self.IR.update(pos,angle,self.r)

            
         
# *****************************************************************
# Simple Robot class with two arms of any number of joints

class NaoRobot:
    def __init__(self, position=(0,0), name="simple", bMatplotlib=False, bTwoArms=True, collisionGroup=None, bOppositeArms=False):
        global nao
        nao = self

        self.ini_pos = position
        x,y = position[0], position[1]
        self.salient = []
        self.bTwoArms = bTwoArms
        self.body_pos = (x,y)
        w = 0.4
        if(bOppositeArms): w = 0.5

        if(not bOppositeArms and bTwoArms): 
            createBox((x-w/2.0,y), w = w/2.0, h = w/1.8, bDynamic=False, bMatplotlib = bMatplotlib, collisionGroup=-1)
            createBox((x+w/2.0,y), w = w/2.0, h = w/1.8, bDynamic=False, bMatplotlib = bMatplotlib, collisionGroup=-1)

        bShrink = False
        length = 1
        self.nparts = 2
        if(name == "human"): 
            self.nparts = 3
            length = 1.0 / 1.5
            bShrink = True

        self.arms = []

        if(bOppositeArms == False):
            self.arms.append(Arm(bLateralize = 1, hdiv = 1, nparts=self.nparts, position=(x-w,y), length = length, name=name, bMatplotlib = bMatplotlib, bShrink = bShrink, collisionGroup=collisionGroup))
            if(bTwoArms): 
                self.arms.append(Arm(bLateralize = 2, hdiv = 1, nparts=self.nparts, position=(x+w,y), length = length, name=name, bMatplotlib = bMatplotlib, bShrink = bShrink, collisionGroup=collisionGroup))
        else:
            arm1 = Arm(position=(x,y), bLateralize = 0, hdiv = 1, nparts=self.nparts, length = length, name=name, bMatplotlib = bMatplotlib, bShrink = bShrink, collisionGroup=collisionGroup)
            arm2 = Arm(position=(x,y+3), signDir=-1, bLateralize = 0, hdiv = 1, nparts=self.nparts, length = length, name=name, bMatplotlib = bMatplotlib, bShrink = bShrink, collisionGroup=collisionGroup)
            self.arms.append(arm1)
            self.arms.append(arm2)


    def getMotorSpeeds(self):
        speeds = []
        for arm in self.arms: speeds += arm.getMotorSpeeds()
        return speeds

    def getJointLimits(self, iarm = -1):
        if(iarm == -1):
            joints = []
            for arm in self.arms: joints += arm.getJointLimits() 
            return joints

        if(iarm >= 2): print "getJointLimits Invalid iarm", iarm
        return self.arms[iarm].getJointLimits()

    def getJointAngles(self, iarm = -1):
        m = []
        if(iarm < 0):
            m += self.arms[0].getJointAngles()
            if(self.bTwoArms): m += self.arms[1].getJointAngles()
            return m
        thearm = self.arms[iarm]
        return thearm.getJointAngles()

    def getFinalPos(self, iarm = -1):
        if(iarm<0): return self.arms[0].getFinalPos() + self.arms[1].getFinalPos() 
        else: return self.arms[iarm].getFinalPos()

    def gotoTargetJoints(self, t = [0,0,0,0,0,0], iarm = -1 ):
        l = len(t)/2
        if(iarm==1 and not self.bTwoArms): return []
        elif(iarm<0):
            self.arms[0].gotoTargetJoints(t[:l])
            if(self.bTwoArms):
                self.arms[1].gotoTargetJoints(t[l:])
        else:
            self.arms[iarm].gotoTargetJoints(t)
        self.update()
        return self.getFinalPos(iarm=iarm)


    def setTargetJoints(self, t = [1,-1], iarm = -1):
        if(iarm<0):
            if(self.bTwoArms):
                l = len(t)/2
                self.arms[0].setTargetJoints(t[0:l])
                self.arms[1].setTargetJoints(t[l:])
            else:             
                self.arms[0].setTargetJoints(t)
        else:
            if(not self.bTwoArms and iarm==1): return
            self.arms[iarm].setTargetJoints(t)

    def restPosition(self,online=True, iarms=[0,1], otherarm=-1):
        if(otherarm>=0): 
            if(otherarm==0): iarms=[1]
            if(otherarm==1): iarms=[0]
        da = self.m_maxs()[0]
        for iarm in iarms:
            if(online):
                if(iarm==1): self.setTargetJoints([-da]+[0]*(self.nparts-1),iarm=1) 
                if(iarm==0): self.setTargetJoints([da]+[0]*(self.nparts-1),iarm=0)
            else:
                if(iarm==1): self.gotoTargetJoints([-da]+[0]*(self.nparts-1),iarm=1) 
                if(iarm==0): self.gotoTargetJoints([da]+[0]*(self.nparts-1),iarm=0)
            self.stop(iarm)
            
    def stop(self,iarm=-1):
        iarms = [0,1]
        if(iarm>=0): iarms = [iarm]
        for iarm in iarms:
            if(not self.bTwoArms and iarm==1): return
            self.arms[iarm].stop()        
        

    def deltaMotorUpdate(self,dm=[]):
        self.deltaMotor(dm)
        for i in range(25):
            self.update()
            world.Step(TIME_STEP, vel_iters, pos_iters)
            world.ClearForces()
        return self.getFinalPos()

    def deltaMotor(self,dm=[],iarm=-1):
        if(iarm < 0):
            if(len(dm)==0):
                l = 2*self.arms[0].nparts 
                dm = [round(2*r-1,2) for r in np.random.rand(l)]
            i = len(dm)/2
            self.arms[0].deltaMotor(dm[0:i])        
            self.arms[1].deltaMotor(dm[i:])
        else:
            if(len(dm)==0):
                l = self.arms[iarm].nparts 
                dm = [round(2*r-1,2) for r in np.random.rand(l)]
            self.arms[iarm].deltaMotor(dm)    

    def update(self, iarm=-1):
        sum = 0
        if(iarm<0):
            self.salient = []
            for iarm,a in enumerate(self.arms): 
                sum += self.arms[iarm].update()
                self.salient +=  self.arms[iarm].getSalient()
        else:
            sum = self.arms[iarm].update()
        return sum

    def getSalient(self):
        return self.salient

    def m_mins(self): 
        joint_limits = self.getJointLimits()
        return [j[0] for j in joint_limits]            

    def m_maxs(self): 
        joint_limits = self.getJointLimits()
        return [j[1] for j in joint_limits]            

    def s_mins(self): return [-3.0,  -2.0]
    def s_maxs(self): return [3.0,  3.0]


    def dm_mins(self):  return [-1]*len(self.getJointLimits())
    def dm_maxs(self):  return [1]*len(self.getJointLimits())
    def ds_mins(self):  return [-1]*4
    def ds_maxs(self):  return [1]*4
    def rest_position(self): return self.m_mins()



# *****************************************************************
# Arm class of any parts and adding a joint extra hand if wanted
       
class CartPole:

    def __init__(self, position=(0,0), name="simple", bMatplotlib = False, d = 1, bHand = 0, collisionGroup=None):
        global bDebug
        self.name = name
        self.ini_pos = position
        self.salientMode = "all"
        self.circle = createCircle(position, r=d*0.6, bDynamic=True, density=1, bMatplotlib = True, name="wheel")
        self.box = createBox( (position[0],position[1]+d*1.9), d*0.2, d*2, bDynamic=True, density=1)
        self.joint = myCreateRevoluteJoint(self.circle,self.box,position,iswheel=True)    
        self.bHand = bHand

        self.IR = IR(1)
        self.box.userData["name"] = name 
        self.box.userData["nIR"]=1

        if(name == "cartLeft"): self.IR.IRAngles = [0] 
        else: self.IR.IRAngles = [np.pi]
            
        self.box.userData["IRAngles"] = self.IR.IRAngles
        self.box.userData["IRValues"] = self.IR.IRValues

        if(bHand>0): 
            body = self.box
            h = d*0.15
            w = d*0.4
            pos = (2*w-d*0.2,0)
            if(bHand == 2): pos = (-2*w+d*0.2,0)
            fixtureDef = createBoxFixture(pos, width=w, height=h, collisionGroup = collisionGroup)
            fixture = body.CreateFixture(fixtureDef)


        self.motor_speed = 0
        self.angle = 0

    def resetPosition(self):
        ipos = self.ini_pos
        self.motor_speed = 0
        self.angle = 0
        self.joint.motorSpeed = 0
        self.joint.torque = 0
        self.box.linearVelocity = [0,0]
        self.box.angularVelocity = 0
        self.box.angle = 0
        self.box.position = (ipos[0],ipos[1]+1.5)

        self.circle.linearVelocity = [0,0]
        self.circle.angularVelocity = 0
        self.circle.angle = 0
        self.circle.position = (ipos[0],ipos[1])

    def getChestPos(self):
        body = self.box
        shape = body.fixtures[0].shape
        vertices=[body.transform*v for v in shape.vertices]
        if(body.userData["name"] == "cartLeft"):
            a,b = vertices[2],vertices[1]
            chestpos = a+(b-a)/3.8
        else:
            a,b = vertices[3],vertices[0]
            chestpos = a+(b-a)/3.8   
        return chestpos
    
    def getBodyPos(self):
        body = self.box
        shape = body.fixtures[1].shape
        verticesHand=[body.transform*v for v in shape.vertices]

        shape = body.fixtures[0].shape
        vertices=[body.transform*v for v in shape.vertices]

        if(self.bHand == 2): 
            d = vertices[2] - vertices[1]
            d = (d[0]/12.,d[1]/12.)
            p = (verticesHand[1] + verticesHand[2])/2.0 + d 
        else: 
            d = vertices[2] - vertices[1]
            d = (d[0]/12.,d[1]/12.)
            p = (verticesHand[3] + verticesHand[0])/2.0 + d

        ret = [p[0], p[1]]
        ret = [round(e,2) for e in ret]
        return ret


    def setMotorSpeed(self,speed):  
        self.joint.motorSpeed = speed

    def getAngle(self):
        return self.box.angle

    def getPosition(self):
        return self.box.position[0]

    def getVelocity(self):
        return self.box.linearVelocity

    def update(self):
        self.angle = self.getAngle()
        self.IR.update(self.getChestPos(), self.angle)


