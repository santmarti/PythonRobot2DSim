# add -*- coding: utf-8 -*-

import sys
import time
import numpy as np
import Box2D
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import itertools
import VectorFigUtils 
from VectorFigUtils import drawBox, drawCircle, makeFigure, vnorm, vrotate, dist, vangleSign, computePointsAngle


# ********************************************
# World Related Globals

start_time = time.time()
world = Box2D.b2World(gravity=[0.0, -0.001]) # normal gravity -9.8

arm = None
nao = None
ground = 0
TARGET_FPS=500                  # Affects framerate (real max 50) and simulation speed                      
TIME_STEP=1.0/TARGET_FPS        # as timestep is the inverse of it, setting it to a
                                # real possible frame rate like 30 makes the sim unfeasible
vel_iters, pos_iters = 40, 40   # To look also world.Step(TIME_STEP,vel_iters,pos_iters)
SPEED_JOINT = 14
bDebug = True


# ********************************************
# Custom Contact Filters

class DefaultContactFilter(Box2D.b2ContactFilter):
    def __init__(self):
        Box2D.b2ContactFilter.__init__(self)

    def ShouldCollide(self, shape1, shape2):
        # Implements the default behavior of b2ContactFilter in Python
        filter1 = shape1.filterData
        filter2 = shape2.filterData
        if filter1.groupIndex == filter2.groupIndex and filter1.groupIndex != 0:
            return filter1.groupIndex > 0
        collides = (filter1.maskBits & filter2.categoryBits) != 0 and (filter1.categoryBits & filter2.maskBits) != 0
        return collides

class CustomContactFilter(Box2D.b2ContactFilter):
    def __init__(self):
        Box2D.b2ContactFilter.__init__(self)

    def ShouldCollide(self, shape1, shape2):
        return 0

def collisions(bOn = True):
    if(bOn): world.contactFilter = DefaultContactFilter()
    else: world.contactFilter = CustomContactFilter()

# ********************************************
# RayCast Collisions : Robots, Epuck

class RayCastCallback(Box2D.b2RayCastCallback):

    def __init__(self, **kwargs):
        super(RayCastCallback, self).__init__()
        self.fixture = None

    def ReportFixture(self, fixture, point, normal, fraction):
        self.fixture = fixture
        self.point = point
        self.normal = normal
        return fraction

class fwQueryCallback(Box2D.b2QueryCallback):

    def __init__(self, p):
        super(fwQueryCallback, self).__init__()
        self.point = p
        self.fixture = None

    def ReportFixture(self, fixture):
        body = fixture.body
        if body.type == Box2D.b2_dynamicBody:
            inside = fixture.TestPoint(self.point)
            if inside:
                self.fixture = fixture
                # We found the object, so stop the query
                return False
        # Continue the query
        return True

def queryPoint(p):
    aabb = Box2D.b2AABB(lowerBound=(p[0]-0.001, p[1]-0.001),
                  upperBound=(p[0]+0.001, p[1]+0.001))

    # Query the world for overlapping shapes.
    query = fwQueryCallback(p)
    world.QueryAABB(query, aabb)

    if query.fixture:
        body = query.fixture.body


# ********************************************
# Basic Creation Box2D Functions that can use the global figure to plot
fig,ax = None,None

def createGlobalFigure():
    global fig,ax
    if(fig == None):
        plt.ioff()
        fig = plt.figure()
        ax = plt.axes(xlim=VectorFigUtils.x_lim, ylim=VectorFigUtils.y_lim)


def step():
    global world, TIME_STEP, vel_iters, pos_iters, arm
    world.Step(TIME_STEP, vel_iters, pos_iters)
    world.ClearForces()


def plotWorld(ax, alpha=0.3, nao=None, obj=None, bDrawGround=False, color='b', centers=[], specials=[], cradius=0.1, ccolor='r', label='_'):
    global world
    if(nao and obj):
        pobj = [obj.position[0], obj.position[1]]
        pnao = nao.ini_pos
    #ax.plot([pobj[0],pnao[0]], [pobj[1],pnao[1]], linestyle='--', color='g', lw=2)
    for body in world.bodies:
        name = body.userData["name"]
        for fixture in body.fixtures:
            shape = fixture.shape
            if(body.active):
                if(isinstance(shape,Box2D.b2PolygonShape)): 
                    if(name == "boxA"): drawBox2D(ax,body,fixture, color='g',alpha=1)
                    elif(name == "boxB"): drawBox2D(ax,body,fixture, color='r',alpha=1)   
                    elif(name == "bar"): drawBox2D(ax,body,fixture, color=color,alpha=0.1)                                                
                    else:
                        if(fixture.density == 0.0): drawBox2D(ax,body,fixture,color=color,alpha=0.25)
                        else:                       drawBox2D(ax,body,fixture,color=color,alpha=alpha)    
                if(isinstance(shape,Box2D.b2CircleShape)): 
                    if(name == "epuck"): drawEpuck(ax,shape,body)
                    elif(name == "reward"): drawCircle(ax,body.position,shape.radius,color=[0.5,0.7,0.3])
                    elif(name == "toy"): drawCircle(ax,body.position,shape.radius)
                    else: drawWheel(ax,shape,body)
            else:  
                if(isinstance(shape,Box2D.b2PolygonShape)): drawBox2D(ax,body,fixture,color=color,alpha=0.25,fill=False,linestyle='dashed')
                if(isinstance(shape,Box2D.b2CircleShape)):  drawCircle(ax,body.position,0.3,fill=False,linestyle='dashed')

    if(len(centers)>0): plotVectors(ax,centers, cradius=cradius, specials=specials, ccolor=ccolor, label=label)
    plt.grid()



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

# function specific of an Animation: animateWorld function
def init():
    circles, = ax.plot([], [], 'bo', ms=5)
    lines, = ax.plot([], [], '-', lw=2)
    lines.set_data([], [])
    circles.set_data([], [])
    return lines, circles

def animateWorld():
    global world, fig, ax
    createGlobalFigure()
    anim = animation.FuncAnimation(fig, frameWorld, init_func=init,frames=150, interval=20, blit=True)
    return anim    

        

# ********************************************
# Creation and plotting Utils


def printProgress(i=0, val=None, valp=None):
    if(i%10==0): 
        print ".",
        if(i%50==0): 
            print i,
            if(i>0 and val!=None and valp==None): print "(",val,")",
            elif(i>0 and val!=None and valp!=None): print "(",val,",",valp,")",
        sys.stdout.flush()

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

def drawEpuck(ax,shape,body):
    r = shape.radius
    pos = body.position
    drawCircle(ax,pos,r)
    irangles = body.userData["frontIRAngles"]
    nir = len(irangles)
    for a in [-np.pi/4,np.pi/4]:
        v = vrotate((1,0),body.angle+a) 
        x = [pos[0],pos[0]+0.99*r*v[0]]
        y = [pos[1],pos[1]+0.99*r*v[1]]
        ax.plot(x,y, linestyle='-', color='b', lw=1)

    for k in range(nir):
        v = vrotate((1,0),body.angle + irangles[k])
        c = pos+[0.97*r*v[0],0.97*r*v[1]]
        value = body.userData["frontIRValues"][k]
        if(value > 1): value = 1
        drawCircle(ax,c,r=0.07,color=[1-value,value,0])


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


def myCreateDistanceJoint(bodyA,bodyB,dx=0):
    global world

    pA = (bodyA.worldCenter[0]+dx, bodyA.worldCenter[1])
    pB = (bodyB.worldCenter[0], bodyB.worldCenter[1])
    joint = world.CreateDistanceJoint(
            bodyA=bodyA, 
            bodyB=bodyB, 
            anchorA=pA,
            anchorB=pB,
            collideConnected = False,
            )
    #joint.frequencyHz = 0.000001
    #joint.dampingRatio = 0.5
    return joint


def myCreateLinearJoint(bodyA,bodyB,force=100,lowerTranslation = -0.2,upperTranslation = 0):
    global world
    center = (bodyA.worldCenter + bodyB.worldCenter)/2.0
    joint = world.CreatePrismaticJoint(
            bodyA=bodyA, 
            bodyB=bodyB, 
            anchor=center,
            axis = (1,0),
            enableLimit = True,
            lowerTranslation = lowerTranslation, 
            upperTranslation = upperTranslation,
            motorSpeed = force,
            maxMotorForce = force,
            enableMotor = True,
            collideConnected = False,
            )
    return joint


def myCreateRevoluteJoint(bodyA,bodyB,anchor,lowerAngle = -0.7 * np.pi, upperAngle = 0.7 * np.pi,iswheel=False):
    global world
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


def createGround(position=[0,-20], bMatplotlib = True):
    global world
    groundBodyDef = Box2D.b2BodyDef()
    groundBodyDef.position = Box2D.b2Vec2(0,-20)   
    groundBody = world.CreateBody(groundBodyDef)

    groundBox = Box2D.b2PolygonShape()
    groundBox.SetAsBox(100, 10)
    fixture = groundBody.CreateFixturesFromShapes(groundBox,friction=100)
    
    if(bMatplotlib):
        createGlobalFigure()
        shape = fixture.shape
        vertices=[groundBody.transform*v for v in shape.vertices]
        poly = plt.Polygon(vertices, lw=5, edgecolor='b')
        ax.add_patch( poly )

    return groundBody


def createCircle(position, r=0.3, bDynamic=True, density=1, bMatplotlib = True, name=""):
    global world, fig, ax
    bodyDef = Box2D.b2BodyDef()
    fixtureDef = Box2D.b2FixtureDef()
    if bDynamic:
        bodyDef.type = Box2D.b2_dynamicBody
        fixtureDef.density = density
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
    fixture = body.CreateFixture(shape=Box2D.b2CircleShape(radius=r), density=density, friction=200)    
    body.userData = {"name":name}

    if(bMatplotlib): 
        createGlobalFigure()
        fixture.userData = drawCircle(ax,position,r)
    
    return body


def createRope(position, nparts=10, r=0.3, density=1, bMatplotlib = True, name=""):
    global world
    jointList = []
    
    firstBody = createCircle( position, r=r, bMatplotlib=bMatplotlib)
    firstBody.userData["name"]="ropepart"   

    prevBody = firstBody
    pos = position
    for i in range(nparts):
        body = createCircle( pos, r=r, density=density, bMatplotlib=bMatplotlib)
        jointList.append( myCreateDistanceJoint(prevBody,body) )
        pos = (pos[0]+1.55*r, pos[1])
        prevBody = body
    return [firstBody, body], [jointList[0], jointList[-1]]


def createBoxFixture(pos = (0,0), width=1.0, height=1.0, bDynamic=True, density = 1, collisionGroup = None, restitution=None):
    global world
    boxShape = Box2D.b2PolygonShape()
    boxShape.SetAsBox(width, height, pos, 0)    # width, height, position (x,y), angle 
    fixtureDef = Box2D.b2FixtureDef()
    fixtureDef.shape = boxShape

    fixtureDef.friction = 0.3
    if(abs(world.gravity[1]) > 1):
        fixtureDef.restitution = 0.6

    if(restitution != None): fixtureDef.restitution = restitution

    if(collisionGroup!=None): fixtureDef.filter.groupIndex = collisionGroup
    
    if bDynamic: fixtureDef.density = density
    else:       fixtureDef.density = 0            
    return fixtureDef

def createBox(position, w=1.0, h=1.0, wdiv = 1, hdiv = 1, bDynamic=True, density=1, damping = 0, collisionGroup = None, bMatplotlib = True, restitution=None, bCollideNoOne=False, name=""):
    global world
    bodyDef = Box2D.b2BodyDef()
    bodyDef.position = position

    if(abs(world.gravity[1]) > 1):
        bodyDef.linearDamping = damping
        bodyDef.angularDamping = 0
    else:
        bodyDef.linearDamping = 70
        bodyDef.angularDamping = 50

    if bDynamic: bodyDef.type = Box2D.b2_dynamicBody
    else:        bodyDef.type = Box2D.b2_staticBody

    body = world.CreateBody(bodyDef)
    body.userData = {"name":name}

    dw = w / float(wdiv)
    dh = h / float(hdiv)

    for i in range(hdiv):
        for j in range(wdiv):
            x = 2*j*dw + (1-wdiv)*dw
            y = 2*i*dh + (1-hdiv)*dh
            fixtureDef = createBoxFixture((x,y), width=dw, height=dh, bDynamic=bDynamic, density=density, collisionGroup = collisionGroup, restitution=restitution)
            if(bCollideNoOne): fixtureDef.filter.maskBits = 0x0000;
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
    body.userData = {"name":"tri"}

    if(bMatplotlib): 
        createGlobalFigure()
        fixture.userData = drawTri(ax,position,r)
    
    return body

    
def createArm(position = (0,0), nparts = 4, name="simple", bMatplotlib = True, collisionGroup = None, length = 1, bHand = False, hdiv = 1, bLateralize = 0, bShrink = False, signDir=1):
    global world
    jointList = []
    d = 1
    l = length
    lsum = 0
    prevBody = createBox( position, 0.1, 0.1, hdiv = 1, bDynamic=False, collisionGroup=-1)
    prevBody.userData["name"]="armpart"    
    if(signDir < 0): prevBody.userData["name"]="reversearmpart"    
    for i in range(nparts):
        w = l / 10.0
        #pos = tuple(map(sum,zip(position, (0, d*(l*0.5 + lsum)))))
        #anchor = tuple(map(sum,zip(position, (0, d*lsum))))
        pos = tuple(map(sum,zip(position, (0, signDir*d*(l*0.5 + lsum)))))
        anchor = tuple(map(sum,zip(position, (0, signDir*d*lsum))))

        if(i==0): box = createBox( pos, w, l*0.5, hdiv = hdiv, collisionGroup=-1 )
        else: box = createBox( pos, w, l*0.5, damping=500, hdiv = hdiv, collisionGroup=collisionGroup)
        
        box.userData["name"]="armpart"    
        if(signDir < 0): box.userData["name"]="reversearmpart"    

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
            



def vrotate(v, angle, anchor=[0,0]):
    """Rotate a vector `v` by the given angle, relative to the anchor point."""
    x, y = v
    x = x - anchor[0]
    y = y - anchor[1]
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    nx = x*cos_theta - y*sin_theta
    ny = x*sin_theta + y*cos_theta
    nx = nx + anchor[0]
    ny = ny + anchor[1]
    return [nx, ny]

