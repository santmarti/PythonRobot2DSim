import sys
import time
import numpy as np
import Box2D
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
TARGET_FPS=500
TIME_STEP=1.0/TARGET_FPS
vel_iters, pos_iters = 40, 40
SPEED_JOINT = 14
bDebug = True

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

def myCreateLinearJoint(bodyA,bodyB,force=100):
    global world
    center = (bodyA.worldCenter + bodyB.worldCenter)/2.0
    joint = world.CreatePrismaticJoint(
            bodyA=bodyA, 
            bodyB=bodyB, 
            anchor=center,
            axis = (1,0),
            enableLimit = True,
            lowerTranslation = -0.2, 
            upperTranslation = 0,
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


def createBoxFixture(body, pos = (0,0), width=1.0, height=1.0, dynamic=True, collisionGroup = None, restitution=None):
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
    
    if dynamic: fixtureDef.density = 1
    else:       fixtureDef.density = 0            
    return fixtureDef

def createBox(position, w=1.0, h=1.0, wdiv = 1, hdiv = 1, dynamic=True, damping = 0, collisionGroup = None, bMatplotlib = True, restitution=None):
    global world
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
            fixtureDef = createBoxFixture(body, (x,y), width=dw, height=dh, collisionGroup = collisionGroup, restitution=restitution)
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
            

