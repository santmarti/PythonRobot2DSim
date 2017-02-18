import sys
import time
import numpy as np
import itertools
import math
import VectorFigUtils
from Box2DWorld import world, TIME_STEP, vel_iters, pos_iters, createArm, bDebug, SPEED_JOINT
            
# *****************************************************************
# Arm class of any parts and adding a joint extra hand if wanted
       
class Arm:
    size_history  = 50

    def __init__(self, nparts=2, position=(0,0), name="simple", length=1, bHand=False, hdiv=1, bLateralize=0, bShrink=False, collisionGroup=None,signDir=1):
        global arm, bDebug
        arm = self
        self.name = name
        self.pos = position
        self.salientMode = "all"
        self.nparts = nparts
        self.bHand = bHand
        self.jointList = createArm(position, nparts, bLateralize=bLateralize, length=length, bHand=bHand, hdiv=hdiv, name=name, bShrink = bShrink, collisionGroup=collisionGroup,signDir=signDir)

        self.targetJoints = [0] * nparts      # without np multiply equals repeat
        if(bHand): self.targetJoints += [0]   # withou np sum equals concat
        self.salient = []
        self.maxSpeedJoint = SPEED_JOINT
        self.targetMode = False
        self.iforce = -1
        self.speedGain = 12 # 1 unit in environment displacement
        self.history = []

        if(bLateralize==0): self.which = "None"
        elif(bLateralize==1): self.which = "Left"
        elif(bLateralize==2): self.which = "Right"

        if(bDebug):
            if(bLateralize==1): print "Created Left Lateralized Arm Lims: ", self.m_mins(), self.m_maxs()  
            elif(bLateralize==2): print "Created Right Lateralized Arm Lims: ", self.m_mins(), self.m_maxs()  
            else: print "Created NON Lateralized Arm Lims: ", self.m_mins(), self.m_maxs()  

    def setTargetJoints(self, t = [0,0] ):
        self.targetMode = True
        #print "Arm setTargetJoints t,len", t, len(t), "  and with ", self.targetJoints
        for i in range(len(t)):
            self.targetJoints[i] = t[i]


    def dmnorm(self,m):
        mplus=np.array(self.getJointAngles())
        return VectorFigUtils.vnorm(mplus - np.array(m))

    def getJointSpeedNorm(self):
        return VectorFigUtils.vnorm(self.getMotorSpeeds())

    def errorMinWorldLoop(self):
        global world, TIME_STEP, vel_iters, pos_iters
        err = self.update()
        normhist, sumerr, niter = [ 1 ], 1, 0
        while(err > 0.05 and sumerr > 0.01 and niter < 500):
            m = self.getJointAngles()
            err = self.update()
            world.Step(TIME_STEP, vel_iters, pos_iters)
            world.ClearForces()   
            normhist.append(self.dmnorm(m))
            if(len(normhist)>15): normhist.pop(0)
            sumerr = sum(normhist)
            niter += 1
            #if(int(100*(time.time() - start_time)) % 10 == 0):
            #    print self.which, "err", err, "dmnorm", sum(normhist)

        return err

    def gotoTargetJoints(self, t = [0,0] ):
        self.setTargetJoints(t)
        err = self.errorMinWorldLoop()
        #if(err > 0.05): print self.which,"arm gotoTargetJoints",t,"Could not be reached"
        return self.getFinalPos()

    def getJointLimits(self):
        return [(round(j.limits[0],2),round(j.limits[1],2)) for j in self.jointList]

    def getJointAngles(self):
        return [j.angle for j in self.jointList]

    def getJointPositionsXY(self):
        lpos = []
        for i in [-2-j for j in range(self.nparts-1)]:
            p = self.getFinalPos(part=i)
            lpos += [(p[0],p[1])]
        lpos += [(self.pos[0],self.pos[1])]
        return lpos

    def getMotorSpeeds(self):
        return [j.motorSpeed for j in self.jointList]


    def getFinalPos(self, part = -1):
        if(self.bHand and part < -1): part = part - 1
        body = self.jointList[part].bodyB
        shape = body.fixtures[-1].shape
        vertices=[body.transform*v for v in shape.vertices]
        if(part == -1 and self.bHand): # returns the angle when we have hand        
            p = body.position
            ret = [p[0], p[1]]
            u,v = (1,0), (vertices[-2] - vertices[-1]) #vector to compute angle to return
            angle = vangleSign(u,v)            
            ret = [p[0], p[1], angle]
        else:
            if(body.userData["name"] != "reversearmpart"):
                p = (vertices[-1] + vertices[-2])/2.0 # in the middle
            else:
                p = (vertices[0] + vertices[1])/2.0 # if the arm is reversed downside

            ret = [p[0], p[1]]

        ret = [round(e,2) for e in ret]
        return ret

    def updateSalient(self):  #salient points update     
        if(self.salientMode == "all"):
            self.salient = []
            for i in range(self.nparts):
                p = self.getFinalPos(part=-(i+1)) # the hand has the same endpoint
                self.salient += [(p[0],p[1])]
            return
        pf = self.getFinalPos()
        self.salient = [(pf[0],pf[1])]
        if(self.salientMode != "endpoint"):
            pm = self.getFinalPos(part=-2)
            self.salient += [(pm[0],pm[1])]
        return self.salient

    def addHistory(self):
        #hist = np.array(self.getJointAngles())        
        hist = self.getMotorSpeeds()
        hist = [round(e/SPEED_JOINT,2) for e in hist]
        self.history.append(hist)
        if(len(self.history) > Arm.size_history): 
            self.history.pop(0)     

    def update(self):
        ret = 0
        if(self.targetMode): ret = self.updatePID()  # returns the sum of the error of each DOF
    
        self.updateSalient()
        self.addHistory()

        if(not self.targetMode): 
            if(self.iforce > 0):   self.iforce -= 1
            elif(self.iforce == 0): 
                self.iforce = -1
                self.stop()
    
        return ret

    def stop(self):        
        for i,j in enumerate(self.jointList): j.motorSpeed = 0

    def deltaMotorUpdate(self,dm):
        self.deltaMotor(dm)
        for i in range(20):
            self.update()
            world.Step(TIME_STEP, vel_iters, pos_iters)
            world.ClearForces()
        return self.getFinalPos()

    def deltaMotor(self,dm=[]):
        if(len(dm)==0): dm = [round(2*r-1,2) for r in np.random.rand(self.nparts)]
        self.targetMode = False
        self.iforce = 10
        for i,j in enumerate(self.jointList):
            j.motorSpeed = self.speedGain*dm[i]
        return dm

    def updatePID(self): #PID update for position control        
        error_sum = 0
        i = 0
        for j, tangle in zip(self.jointList, self.targetJoints):
            invj = max(1,2-i)            
            maxSpeedJoint = self.maxSpeedJoint/float(invj)
            angleError = j.angle - tangle            
            j.motorSpeed = -maxSpeedJoint * angleError
            error_sum += abs(angleError)
            i += 1
        return error_sum

    def changeSalientMode(self,salientMode):
        self.salientMode = salientMode
        self.updateSalient()

    def getSalient(self):
        return self.salient

    def m_mins(self): 
        joint_limits = self.getJointLimits()
        return [j[0] for j in joint_limits]            

    def m_maxs(self): 
        joint_limits = self.getJointLimits()
        return [j[1] for j in joint_limits]            

    def dm_mins(self):  return [-1]*len(self.getJointLimits())
    def dm_maxs(self):  return [1]*len(self.getJointLimits())

    def s_mins(self): return [-3.0,  -1.0]
    def s_maxs(self): return [3.0,  3.0]

    def ds_mins(self): return [-1.0,  -1.0]
    def ds_maxs(self): return [1.0,  1.0]

    def rest_position(self): return [0]*len(self.getJointLimits())

