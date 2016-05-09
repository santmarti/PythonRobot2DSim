import sys
import time
import numpy as np
import Box2D
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.collections import PolyCollection
import itertools
import math
import VectorFigUtils 
from VectorFigUtils import drawBox, drawCircle, makeFigure, vnorm, vrotate, dist, vangleSign, computePointsAngle

start_time = time.time()
world = Box2D.b2World(gravity=[0.0, -0.001]) # normal gravity -9.8
arm = None
nao = None
ground = 0
TARGET_FPS=500
TIME_STEP=1.0/TARGET_FPS
vel_iters, pos_iters = 40, 40
SPEED_JOINT = 14
bDebug = True

def printProgress(i=0, val=None, valp=None):
    if(i%10==0): 
        print ".",
        if(i%50==0): 
            print i,
            if(i>0 and val!=None and valp==None): print "(",val,")",
            elif(i>0 and val!=None and valp!=None): print "(",val,",",valp,")",
        sys.stdout.flush()

def step():
    global world, TIME_STEP, vel_iters, pos_iters, arm
    world.Step(TIME_STEP, vel_iters, pos_iters)
    world.ClearForces()

def makeFigureWorld(pos=[], angle=None):
    fig,ax = makeFigure()
    if(len(pos)>0 and angle != None): plotSensoryState(ax,pos,angle)
    plotWorld(ax)
    return fig,ax

def drawWheel(ax,shape,body):
    r = shape.radius
    pos = body.transform*shape.pos
    drawCircle(ax,pos,r)
    for a in [0,2*np.pi/3,4*np.pi/3]:
        v = vrotate((1,0),body.angle+a) 
        x = [pos[0],pos[0]+0.99*r*v[0]]
        y = [pos[1],pos[1]+0.99*r*v[1]]
        ax.plot(x,y, linestyle='-', color='b', lw=1)


def plotVectors(ax, centers, specials=[], dirs = [], label='_', cradius=0.1, ccolor='r'):
    for i,p in enumerate(centers):
        dx,dy = -2*cradius,cradius
        if(p[0] >= 0): dx,dy = cradius,cradius/2.0
        if(label != '_'):
            plt.annotate(r'$%s_%d$'%(label,i), xy=p, xytext=(p[0]+dx,p[1]+dy))
        
        if(len(specials)>0):
            if(i in specials): drawCircle(ax,p,cradius,color=ccolor) 
            else: drawCircle(ax,p,cradius-0.05,color='b') 
        else: drawCircle(ax,p,cradius,color=ccolor) 
        
        if(len(dirs)>0):
            d = dirs[i]
            vel = (3*d[0],3*d[1])
            if(VectorFigUtils.vnorm(vel)):
                ax.arrow(p[0], p[1], vel[0], vel[1], head_width=0.07, head_length=0.11, fc='k', ec='k')

def plotWorld(ax, alpha=0.3, nao=None, obj=None, bDrawGround=False, color='b', centers=[], specials=[], cradius=0.1, ccolor='r', label='_'):
    global world
    if(nao and obj):
        pobj = [obj.position[0], obj.position[1]]
        pnao = nao.ini_pos
    #ax.plot([pobj[0],pnao[0]], [pobj[1],pnao[1]], linestyle='--', color='g', lw=2)
    for body in world.bodies:
        for fixture in body.fixtures:
            shape = fixture.shape
            if(body.active):
                if(isinstance(shape,Box2D.b2PolygonShape)): 
                    if(fixture.density == 0.0): drawBox2D(ax,body,fixture,color=color,alpha=0.25)
                    else:                       drawBox2D(ax,body,fixture,color=color,alpha=alpha)    
                if(isinstance(shape,Box2D.b2CircleShape)): 
                    drawWheel(ax,shape,body)
            else:  
                if(isinstance(shape,Box2D.b2PolygonShape)): drawBox2D(ax,body,fixture,color=color,alpha=0.25,fill=False,linestyle='dashed')
                if(isinstance(shape,Box2D.b2CircleShape)):  drawCircle(ax,body.position,0.3,fill=False,linestyle='dashed')

    if(len(centers)>0): plotVectors(ax,centers, cradius=cradius, specials=specials, ccolor=ccolor, label=label)
    plt.grid()

def plotAll(ax, alpha = 0.3, color = 'b', centers=[], specials=[], ccolor='r', cradius = 0.1, label='_', xlabel="", ylabel="", title="", mainfont=16):
    plotWorld(ax,alpha=alpha, color=color, centers=centers, specials=specials, cradius=cradius, ccolor=ccolor, label=label)
    VectorFigUtils.decorate(xlabel=xlabel,ylabel=ylabel,title=title,mainfont=mainfont)    


def plotSensoryState(ax,pos,angle=None):
    ax.plot(*pos, marker='o', color='red')
    if(angle):
        p1,p2,p3 = computePointsAngle(pos,angle) 
        ax.plot([p1[0],p2[0]], [p1[1],p2[1]], linestyle='--', color='g', lw=3)
        ax.plot([pos[0],p3[0]], [pos[1],p3[1]], linestyle='-', color='g', lw=2)


def drawBox2D(ax, body, fixture, alpha = 0.5, color = 'b', fill=True, linestyle='solid'):
    shape = fixture.shape
    vertices=[body.transform*v for v in shape.vertices]
    poly = drawBox(ax,vertices,alpha=alpha,color=color,fill=fill,linestyle=linestyle)
    return poly


def myCreateRevoluteJoint(bodyA,bodyB,anchor,lowerAngle = -0.7 * np.pi, upperAngle = 0.7 * np.pi,iswheel=False):
    if(not iswheel):
        return world.CreateRevoluteJoint(
                bodyA=bodyA, 
                bodyB=bodyB, 
                anchor=anchor,
                lowerAngle = lowerAngle, # angle limits lower and upper
                upperAngle = upperAngle, 
                enableLimit = True,
                maxMotorTorque = 1000.0,
                motorSpeed = 0.0,
                enableMotor = True,
                collideConnected = False,
                )
    else:
        return world.CreateRevoluteJoint(
                bodyA=bodyA, 
                bodyB=bodyB, 
                anchor=anchor,
                maxMotorTorque = 1000.0,
                motorSpeed = 0.0,
                enableMotor = True,
                )


# ********************************************
# Basic Creation Box2D Functions that can use the global figure to plot
fig,ax = None,None

def createGlobalFigure():
    global fig,ax
    if(fig == None):
        plt.ioff()
        fig = plt.figure()
        ax = plt.axes(xlim=VectorFigUtils.x_lim, ylim=VectorFigUtils.y_lim)

def createGround(position=[0,-20], bMatplotlib = True):
    global world, fig, ax
    groundBodyDef = Box2D.b2BodyDef()
    groundBodyDef.position = Box2D.b2Vec2(0,-20)   
    groundBody = world.CreateBody(groundBodyDef)

    groundBox = Box2D.b2PolygonShape()
    groundBox.SetAsBox(100, 10)
    fixture = groundBody.CreateFixturesFromShapes(groundBox)
    
    if(bMatplotlib):
        createGlobalFigure()
        shape = fixture.shape
        vertices=[groundBody.transform*v for v in shape.vertices]
        poly = plt.Polygon(vertices, lw=5, edgecolor='b')
        ax.add_patch( poly )

    return groundBody

def createCircle(position, r=0.3, dynamic=True, bMatplotlib = True):
    global world, fig, ax
    bodyDef = Box2D.b2BodyDef()
    fixtureDef = Box2D.b2FixtureDef()
    if dynamic:
        bodyDef.type = Box2D.b2_dynamicBody
        fixtureDef.density = 1
    else:
        bodyDef.type = Box2D.b2_staticBody
        fixtureDef.density = 0

    bodyDef.position = position

    if(abs(world.gravity[1]) > 1):
        bodyDef.linearDamping = 0.1
        bodyDef.angularDamping = 0.1
    else:
        bodyDef.linearDamping = 70
        bodyDef.angularDamping = 30

    body = world.CreateBody(bodyDef)
    fixture = body.CreateFixture(shape=Box2D.b2CircleShape(radius=r), density=1.0, friction=0.3)
    
    if(bMatplotlib): 
        createGlobalFigure()
        fixture.userData = drawCircle(ax,position,r)
    
    return body


def createBoxFixture(body, pos = (0,0), width=1.0, height=1.0, dynamic=True, collisionGroup = None):
    global world
    boxShape = Box2D.b2PolygonShape()
    boxShape.SetAsBox(width, height, pos, 0)    # width, height, position (x,y), angle 
    fixtureDef = Box2D.b2FixtureDef()
    fixtureDef.shape = boxShape

    fixtureDef.friction = 0.3
    if(abs(world.gravity[1]) > 1):
        fixtureDef.restitution = 0.6

    if(collisionGroup!=None): fixtureDef.filter.groupIndex = collisionGroup
    
    if dynamic: fixtureDef.density = 1
    else:       fixtureDef.density = 0            
    return fixtureDef

def createBox(position, w=1.0, h=1.0, wdiv = 1, hdiv = 1, dynamic=True, damping = 0, collisionGroup = None, bMatplotlib = True):
    global world, ax
    bodyDef = Box2D.b2BodyDef()
    bodyDef.position = position

    if(abs(world.gravity[1]) > 1):
        bodyDef.linearDamping = damping
        bodyDef.angularDamping = 0
    else:
        bodyDef.linearDamping = 70
        bodyDef.angularDamping = 50

    if dynamic: bodyDef.type = Box2D.b2_dynamicBody
    else:       bodyDef.type = Box2D.b2_staticBody
    body = world.CreateBody(bodyDef)

    dw = w / float(wdiv)
    dh = h / float(hdiv)

    for i in range(hdiv):
        for j in range(wdiv):
            x = 2*j*dw + (1-wdiv)*dw
            y = 2*i*dh + (1-hdiv)*dh
            fixtureDef = createBoxFixture(body, (x,y), width=dw, height=dh, collisionGroup = collisionGroup)
            fixture = body.CreateFixture(fixtureDef)
            if(bMatplotlib): 
                createGlobalFigure()
                fixture.userData = drawBox2D(ax,body,fixture)
    return body


def createTri(position, r=0.3, dynamic=True, bMatplotlib = True):
    global world, fig, ax
    bodyDef = Box2D.b2BodyDef()
    fixtureDef = Box2D.b2FixtureDef()
    if dynamic:
        bodyDef.type = Box2D.b2_dynamicBody
        fixtureDef.density = 1
    else:
        bodyDef.type = Box2D.b2_staticBody
        fixtureDef.density = 0

    bodyDef.position = position
    bodyDef.linearDamping = 70
    bodyDef.angularDamping = 50
    body = world.CreateBody(bodyDef)
    v = [(-r,-r),(0,r),(r,-r)]
    fixture = body.CreateFixture(shape=Box2D.b2PolygonShape(vertices=v), density=1.0, friction=0.3)
    
    if(bMatplotlib): 
        createGlobalFigure()
        fixture.userData = drawTri(ax,position,r)
    
    return body



def init():
    circles, = ax.plot([], [], 'bo', ms=5)
    lines, = ax.plot([], [], '-', lw=2)
    lines.set_data([], [])
    circles.set_data([], [])
    return lines, circles

def frameWorld(i):      # animation function.  This is called sequentially
    global world, TIME_STEP, vel_iters, pos_iters, ax, fig
                
    plotWorld(ax,nao)
                                                 
    if(arm): arm.update()
    world.Step(TIME_STEP, vel_iters, pos_iters)
    world.ClearForces()
    return []

def animateWorld():
    global world, fig, ax
    createGlobalFigure()
    anim = animation.FuncAnimation(fig, frameWorld, init_func=init,frames=150, interval=20, blit=True)
    return anim
    
def createArm(position = (0,0), nparts = 4, name="simple", bMatplotlib = True, collisionGroup = None, length = 1, bHand = False, hdiv = 1, bLateralize = 0, bShrink = False):
    global world
    jointList = []
    d = 1
    l = length
    lsum = 0
    prevBody = createBox( position, 0.1, 0.1, hdiv = 1, dynamic=False, collisionGroup=-1)
    for i in range(nparts):
        w = l / 10.0
        pos = tuple(map(sum,zip(position, (0, d*(l*0.5 + lsum)))))
        anchor = tuple(map(sum,zip(position, (0, d*lsum))))

        if(i==0): box = createBox( pos, w, l*0.5, hdiv = hdiv, collisionGroup=-1 )
        else: box = createBox( pos, w, l*0.5, damping=500, hdiv = hdiv, collisionGroup=collisionGroup)

        if(bLateralize == 0): j = myCreateRevoluteJoint(prevBody,box,anchor)
        elif(bLateralize == 1): 
            if(i == 0):           j = myCreateRevoluteJoint(prevBody,box,anchor,lowerAngle=-0.4 * np.pi,upperAngle=0.4 * np.pi)    
            elif(i == nparts-1):  j = myCreateRevoluteJoint(prevBody,box,anchor,lowerAngle=-0.87 * np.pi,upperAngle=0.2 * np.pi)   # added wrist like + redundancy 
            else:                 j = myCreateRevoluteJoint(prevBody,box,anchor,lowerAngle=-0.87 * np.pi,upperAngle=0)    

        elif(bLateralize == 2): 
            if(i == 0):           j = myCreateRevoluteJoint(prevBody,box,anchor,lowerAngle=-0.4 * np.pi,upperAngle=0.4 * np.pi)    
            elif(i == nparts-1):  j = myCreateRevoluteJoint(prevBody,box,anchor,lowerAngle=-0.2 * np.pi,upperAngle=0.87 * np.pi)    
            else:                 j = myCreateRevoluteJoint(prevBody,box,anchor,lowerAngle=0,upperAngle=0.87 * np.pi)    

        jointList.append(j)
        prevBody = box
        lsum += l
        if(bShrink): l /= 1.1
 
    if(bHand):
        y = d*l*nparts
        pos = tuple(map(sum,zip(position, (0, y))))
        anchor = pos
        box = createBox( pos, l*0.2, w*0.6 )
        j = myCreateRevoluteJoint(prevBody,box,anchor)  
        jointList.append(j)
        
    return jointList
            


# *****************************************************************
# Arm class of any parts and adding a joint extra hand if wanted
       
class Arm:
    size_history  = 50

    def __init__(self, nparts=2, position=(0,0), name="simple", bMatplotlib = False, length = 1, bHand = False, hdiv = 1, bLateralize = 0, bShrink = False, collisionGroup=None):
        global arm, bDebug
        arm = self
        self.name = name
        self.pos = position
        self.salientMode = "all"
        self.nparts = nparts
        self.bHand = bHand
        self.jointList = createArm(position, nparts, bLateralize = bLateralize, bMatplotlib = bMatplotlib, length = length, bHand = bHand, hdiv = hdiv, name=name, bShrink = bShrink, collisionGroup=collisionGroup)
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
            p = (vertices[-1] + vertices[-2])/2.0 # in the middle
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


# *****************************************************************
# Simple robot class

class NaoRobot:
    def __init__(self, position=(0,0), name="simple", bMatplotlib=False, bTwoArms=True, collisionGroup=None):
        global nao
        nao = self

        self.ini_pos = position
        x,y = position[0], position[1]
        self.salient = []
        self.bTwoArms = bTwoArms
        w = 0.4
        self.body_pos = (x,y)
        if(bTwoArms): 
            createBox((x-w/2.0,y), w = w/2.0, h = w/1.8, dynamic=False, bMatplotlib = bMatplotlib, collisionGroup=-1)
            createBox((x+w/2.0,y), w = w/2.0, h = w/1.8, dynamic=False, bMatplotlib = bMatplotlib, collisionGroup=-1)

        bShrink = False
        length = 1
        self.nparts = 2
        if(name == "human"): 
            self.nparts = 3
            length = 1.0 / 1.5
            bShrink = True

        self.arms = []
        self.arms.append(Arm(bLateralize = 1, hdiv = 1, nparts=self.nparts, position=(x-w,y), length = length, name=name, bMatplotlib = bMatplotlib, bShrink = bShrink, collisionGroup=collisionGroup))
        if(bTwoArms): self.arms.append(Arm(bLateralize = 2, hdiv = 1, nparts=self.nparts, position=(x+w,y), length = length, name=name, bMatplotlib = bMatplotlib, bShrink = bShrink, collisionGroup=collisionGroup))

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
# Experimental Setup Class : NaoRobot class plus walls plus object
# *****************************************************************

class ExpSetupNao:

    max_motor_speed = 30

    def __init__(self, pos_obj = (0,1.3), pos_nao = (0,0), obj_type = "circle", salientMode = "center", name="simple", debug = False, bTwoArms=True, bSelfCollisions=True):
        global bDebug
        bDebug = debug        
        print "-------------------------------------------------"
        print "Created Exp Bimanual Setup ", name, "Debug: ", bDebug
        self.name = name
        self.dm_lim = 1
        self.v_lim = 0.3

        self.salient = []
        self.haptic = []
        self.obj_type = obj_type

        self.addWalls(pos_nao)

        if(bSelfCollisions): collisionGroup=None
        else: collisionGroup=-1

        self.nao = NaoRobot(pos_nao,name=name,bTwoArms=bTwoArms,collisionGroup=collisionGroup)
        #self.human = NaoRobot( (pos_nao[0],pos_nao[1]+4) )

        self.objs = []
        self.objs.append( createCircle(pos_obj, r=0.45, bMatplotlib=False) )
        self.objs.append( createTri(pos_obj, r=0.45, bMatplotlib=False) )
        self.objs.append( createBox(pos_obj, wdiv = 1, hdiv = 1, w = 0.35, h = 0.35, bMatplotlib=False) )

        self.target_objs = []
        self.target_objs.append( createCircle(pos_obj, r=0.45, bMatplotlib=False) )
        self.target_objs.append( createTri(pos_obj, r=0.45, bMatplotlib=False) )
        self.target_objs.append( createBox(pos_obj, wdiv = 1, hdiv = 1, w = 0.35, h = 0.35, bMatplotlib=False) )
        for t in self.target_objs:
            t.active = False
            t.position = [2,4]

        x = -2
        for o in self.objs: o.position, x = (x,4), x+1

        if(obj_type == "circle"): self.obj,self.target_obj = self.objs[0], self.target_objs[0]
        elif(obj_type == "box"):  self.obj,self.target_obj = self.objs[2], self.target_objs[2]

        self.ini_obj_pos = pos_obj

        self.changeSalientMode(salientMode)        
        if(bDebug):
            print "Exp Setup created", self.salientMode, "salient points: ", self.salient 

    def addWalls(self,pos, bMatplotlib=False): # WALL LIMITS of the world               
        x,y = pos 
        wl = 0.2
        h = (5+1)/2.0
        createBox((x,y-1), w = 3, h = wl, dynamic=False, bMatplotlib = bMatplotlib)
        createBox((x,y+5), w = 3, h = wl, dynamic=False, bMatplotlib = bMatplotlib)
        createBox((x-3-wl,y+h-1), w = wl, h = 2.8, dynamic=False, bMatplotlib = bMatplotlib)
        createBox((x+3+wl,y+h-1), w = wl, h = 2.8, dynamic=False, bMatplotlib = bMatplotlib)

    def setTargetObj(self,pos,angle=0):
        self.target_obj.position = pos 
        self.target_obj.angle = angle 

    def changeSalientMode(self,salientMode):
        self.salientMode = salientMode
        self.update()

    def start(self,obj_pos=[]):
        if(obj_pos == []): self.obj.position = self.ini_obj_pos
        else: self.obj.position = obj_pos
        self.nao.restPosition(online=False)

    def getSalient(self):
        return self.salient

    def getSalientType(self,i):
        self.update()
        narms = len(self.nao.arms)
        leftsalient = len(self.nao.arms[0].salient)
        if(i<leftsalient): return "left"
        if(narms > 1 and i < 2*leftsalient): return "right"
        return "obj"

    def updateSalient(self):
        b = self.obj.position
        b = [round(e,2) for e in b]
        body = self.obj
        shape = self.obj.fixtures[-1].shape

        naosalient = self.nao.getSalient() # copy of the pointer
        self.salient = [s for s in naosalient] 

        if(self.salientMode == "center"):
            self.salient += [(b[0],b[1])]
        elif(self.salientMode == "laterals"):
            if(self.obj_type == "circle"):  
                self.salient += [(b[0],b[1])]
                self.salient += [(b[0]-shape.radius,b[1])]
                self.salient += [(b[0]+shape.radius,b[1])]
            if(self.obj_type == "box"):  
                v=[body.transform*v for v in shape.vertices]
                self.salient += [(b[0],b[1])]
                self.salient += [(v[1] + v[2])/2.0]
                self.salient += [(v[0] + v[3])/2.0]
        elif(self.salientMode == "minimum"):
            self.salient += [(b[0],b[1])]
            if(self.obj_type == "box"):  
                v=[body.transform*v for v in shape.vertices]
                self.salient += [[v[0][0],v[0][1]]]
        elif(self.salientMode == "all"):
            self.salient += [(b[0],b[1])]
            if(self.obj_type == "box"):  
                v=[body.transform*v for v in shape.vertices]
                for p in v: self.salient += [p]
                self.salient += [(v[1] + v[2])/2.0]
                self.salient += [(v[0] + v[3])/2.0]
                self.salient += [(v[2] + v[3])/2.0]
                self.salient += [(v[0] + v[1])/2.0]
 

    def getFinalHaptic(self, arm = 0):
        if(arm==0): i = 0 # first salient is always the end point
        else: i = len(self.nao.arms[0].salient)
        return self.haptic[i]

    def updateHaptic(self):
        if(len(self.haptic) < len(self.salient)): self.haptic = [0]*len(self.salient)
        maxd = 0.5 
        dt = 1.3
        for i,s in enumerate(self.salient):
            sbox = (s[0],s[1])            
            mind = maxd
            if(self.haptic[i] > 0): self.haptic[i] /= dt
            else: self.haptic[i] = 0    
            for c in self.obj.contacts:
                cpos = c.contact.worldManifold.points[0]
                if(vnorm(cpos)<0.01): continue
                d = dist(cpos,sbox) 
                if(d < maxd and d < mind):
                    mind = d
                    h = 1 - d/maxd
                    self.haptic[i] = h



    def update(self,iarm =-1):
        err = self.nao.update(iarm = arm)
        self.updateSalient()
        self.updateHaptic()
        return err

    def getObjLine(self):
        b = self.obj.position
        b = (b[0],b[1])
        n = self.nao.body_pos
        return [n,b]

    def setObjPos(self,p=[],angle = 0):
        if(len(p)==0): p = self.ini_obj_pos
        self.obj.position = p
        self.obj.angle = angle
        for i in range(len(self.haptic)):
            self.haptic[i] = 0
        self.update()

    def getObjPos(self, bAngle = False, PPM = 1):
        self.update()
        p = self.obj.position
        p = [PPM*round(p[0],2),PPM*round(p[1],2)]
        if(bAngle): return p,self.obj.angle
        else:       return p

    def getMotorHistory(self, iarm = 0, t=-1): # one iteration before by default
        arms = self.nao.arms 
        h = arms[iarm].history
        if(len(h) >= abs(t)): return h[t]
        else:  return [0]*arms[iarm].nparts
 

    def deltaMotor(self,dm):
        return self.nao.deltaMotor(dm)


# *****************************************************************
# Arm class of any parts and adding a joint extra hand if wanted
       
class CartPole:

    def __init__(self, position=(0,0), name="simple", bMatplotlib = False, length = 1, bHand = 0, collisionGroup=None):
        global bDebug
        self.name = name
        self.ini_pos = position
        self.salientMode = "all" 
        self.circle = createCircle(position, r=0.6, dynamic=True, bMatplotlib = True)
        self.box = createBox( (position[0],position[1]+1.9), 0.2, 2, dynamic=True)
        self.joint = myCreateRevoluteJoint(self.circle,self.box,position,iswheel=True)    

        if(bHand>0): 
            body = self.box
            h = 0.15
            w = 0.4
            pos = (2*w-0.2,0)
            if(bHand == 2): pos = (-2*w+0.2,0)
            fixtureDef = createBoxFixture(body, pos, width=w, height=h, collisionGroup = collisionGroup)
            fixture = body.CreateFixture(fixtureDef)


    def resetPosition(self):
        ipos = self.ini_pos
        self.joint.motorSpeed = 0
        self.box.linearVelocity = [0,0]
        self.box.angularVelocity = 0
        self.box.angle = 0
        self.box.position = (ipos[0],ipos[1]+1.9)

        self.circle.linearVelocity = [0,0]
        self.circle.angularVelocity = 0
        self.circle.angle = 0
        self.circle.position = (ipos[0],ipos[1])

    def setMotorSpeed(self,speed):
        self.joint.motorSpeed = speed

    def getAngle(self):
        return self.box.angle

    def getPosition(self):
        return self.box.position[0]


# *****************************************************************
# Experimental Setup Class : Dual CartPole holding object
# *****************************************************************

class ExpSetupDualCartPole:

    max_motor_speed = 30

    def __init__(self, salientMode = "center", name="simple", debug = False, bLink = 1, bSelfCollisions=True):
        global world, bDebug
        bDebug = debug        
        print "-------------------------------------------------"
        print "Created Exp Dual Cart Pole Setup ", name, "Debug: ", bDebug

        #world = Box2D.b2World(gravity=[0.0, -10])
        world.gravity = Box2D.b2Vec2(0,-980) 

        self.name = name
        self.salient = []

        self.addWalls([0,0])
        self.carts = [CartPole(position=(-2,0),bHand=1), CartPole(position=(2,0),bHand=2)]
        
        if(bLink == 1): 
            self.link = createBox( (0,3), 1.5, 0.15, dynamic=True)

        if(bSelfCollisions): collisionGroup=None
        else: collisionGroup=-1
      
        if(bDebug):
            print "Exp Setup created", "salient points: ", self.salient 


    def addWalls(self,pos, bMatplotlib=False): # WALL LIMITS of the world               
        x,y = pos 
        wl = 0.2
        h = (5+1)/2.0
        createBox((x,y-1), w = 4+2*wl, h = wl, dynamic=False, bMatplotlib = bMatplotlib)
        #createBox((x,y+5), w = 3, h = wl, dynamic=False, bMatplotlib = bMatplotlib)
        createBox((x-4-wl,y+h-1), w = wl, h = 2.8, dynamic=False, bMatplotlib = bMatplotlib)
        createBox((x+4+wl,y+h-1), w = wl, h = 2.8, dynamic=False, bMatplotlib = bMatplotlib)

    def setMotorSpeed(self,i,speed):
        self.carts[i].setMotorSpeed(speed)

    def resetPosition(self):
        self.ikea.linearVelocity = [0,0]
        self.ikea.angularVelocity = 0
        self.ikea.angle = 0
        self.ikea.position = (0,3)
        for i in [0,1]:
            self.carts[i].resetPosition()

    def getAngles(self):
        return [cart.getAngle() for cart in self.carts]

    def getPositions(self):
        return [cart.getPosition() for cart in self.carts]




