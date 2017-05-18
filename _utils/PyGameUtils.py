import numpy as np
import pygame
import Box2D # The main library
from Box2D.b2 import * # This maps Box2D.b2Vec2 to vec2 (and so on)
import Box2DWorld

SCREEN_WIDTH, SCREEN_HEIGHT, X0, Y0 = 640,480,640/2,480/5

def setScreenSize(w,h):
    global SCREEN_WIDTH, SCREEN_HEIGHT, X0, Y0
    SCREEN_WIDTH, SCREEN_HEIGHT=w,h
    X0 = SCREEN_WIDTH/2 # displacement for python notebook plotting purposes 
    Y0 = SCREEN_HEIGHT/5 # not used for the moment

PPM = 65 # pixel size only for pygame
colors = {staticBody:(255,255,235,235), dynamicBody:(127,127,127,190)}

# ****************************************************************************     
# ----  OSC receiving
server = None
def receiveLeft(addr, tags, data, source):
    global nao
    tj = nao.arms[0].targetJoints
    tj[0] += 0.1
    nao.setTargetJoints([ tj[0], tj[1] ])

def receiveRight(addr, tags, data, source):
    global nao
    tj = nao.arms[0].targetJoints
    tj[0] -= 0.1
    nao.setTargetJoints([ tj[0], tj[1] ])
    
#def iniOSC():
    #global server
    #server = OSCServer( ("" , 1234) )
    #for i in range(2,3):
        #print "/wii/%s/accel/pry"%i
        #server.addMsgHandler("/wii/%s/accel/pry"%i, receiveXYZ)
        #server.addMsgHandler("/wii/%s/button/Left"%i, receiveLeft)
        #server.addMsgHandler("/wii/%s/button/Right"%i, receiveRight)

# ****************************************************************************     
# PyGame Mouse         
# ****************************************************************************     

def getMousePos():
    return pygame.mouse.get_pos()



# ****************************************************************************     
# PyGame Drawing function utils         
# ****************************************************************************     

def draw_polygon(screen, vertices, color = (0,255,0), width=3):
    vertices=[(X0+v[0], -Y0+SCREEN_HEIGHT-v[1]) for v in vertices]
    pygame.draw.polygon(screen, color, vertices, width)
        
def box2d_draw_polygon(screen, polygon, body, fixture, color = [], width=3):
    bDraw = 1
    if(len(color)==0): color = colors[body.type]
    vertices=[(body.transform*v)*PPM for v in polygon.vertices]
    if(body.userData["name"]=="bar"):
        draw_polygon(screen, vertices, (10,10,100), 2)
    elif(body.userData["name"]=="occlusion"):
        bDraw = body.userData["visible"]
        if(bDraw):
            draw_polygon(screen, vertices, (100,10,20), 0)       
            draw_polygon(screen, vertices, (200,10,20), 3)       
    elif(body.userData["name"]=="boxA"):
        draw_polygon(screen, vertices, (100,10,20), 0)       
        draw_polygon(screen, vertices, (200,10,20), 3)       
    elif(body.userData["name"]=="boxB"):
        draw_polygon(screen, vertices, (100,80,0), 0)       
        draw_polygon(screen, vertices, (160,120,0), 3)     
    elif(body.userData["name"] in ["cartLeft","cartRight"]):
        draw_polygon(screen, vertices, color, width)
        shape = body.fixtures[0].shape
        vertices=[body.transform*v for v in shape.vertices]
        if(body.userData["name"] == "cartLeft"):
            a,b = vertices[2],vertices[1]
            chestpos = a+(b-a)/3.8
        else:
            a,b = vertices[3],vertices[0]
            chestpos = a+(b-a)/3.8
        drawIR(screen,chestpos,body.angle,body.userData["nIR"],body.userData["IRAngles"],body.userData["IRValues"])    
    else:
        draw_polygon(screen, vertices, color, width)


def draw_circle(screen, position=(0,0), radius = 1, color=(27,200,7,190), width=0):
    global X0,Y0
    position=(X0+position[0], -Y0+SCREEN_HEIGHT-position[1])
    pygame.draw.circle(screen, color, [int(x) for x in position], radius, width)

def box2d_draw_mycircle(screen, pos, radius, color):
    draw_circle(screen, (pos[0]*PPM,pos[1]*PPM), radius=int(radius*PPM), color=color)

def drawWheel(screen,r,body,color,width):
    pos=body.position
    for a in [0,2*np.pi/3,4*np.pi/3]:
        v = Box2DWorld.vrotate((1,0),body.angle+a) 
        my_draw_line(screen, [pos, pos+[0.99*r*v[0],0.99*r*v[1]]], color=color, width=width)

def drawIR(screen,pos,angle,nir,irangles,irvalues,r=0):
    for k in range(nir):
        v = Box2DWorld.vrotate((1, 0), angle + irangles[k])
        c = pos + [0.98 * r * v[0], 0.98 * r * v[1]]
        value = irvalues[k]
        if(value > 1):
            value = 1
        draw_circle(screen, PPM * c, radius=int(0.1 * PPM), color=(0, 0, 255), width=1)
        color = (255 - 255 * value, 0, 0)
        draw_circle(screen, PPM * c, radius=int(0.08 * PPM), color=color, width=0)


def drawEpuck(screen, r, body, color, width):
    pos=body.position
    for a in [-np.pi/4,np.pi/4]:
        v = Box2DWorld.vrotate((1,0),body.angle+a) 
        c = pos+[0.98*r*v[0],0.98*r*v[1]]
        my_draw_line(screen, [pos, c], color=color, width=width)

    nir = body.userData["nIR"]
    drawIR(screen, pos, body.angle, nir, body.userData["IRAngles"], body.userData["IRValues"], r)
    if(body.userData["nOtherSensors"] > 0):
        d = 0.73 if nir > 0 else 1.0
        drawIR(screen, pos, body.angle, body.userData["nOtherSensors"], body.userData["OtherAngles"], body.userData["OtherValues"], d * r)
    if(body.userData["nRewardSensors"] > 0):
        d = 0.43 if nir > 0 else 0.73
        drawIR(screen, pos, body.angle, body.userData["nRewardSensors"], body.userData["RewardAngles"], body.userData["RewardValues"], d * r)


# When drawing a circle called from
# draw_world(screen) defined in this File
# the info in userData (a map {}) is exploited
# userData["name"] can be "epuck", "reward"
# userData["frontIRValues"] is the array of ir sensor values
def box2d_draw_circle(screen, circle, body, fixture, color=[], width=3):
    if(len(color) == 0): color = colors[body.type]
    r = circle.radius
    pos = body.position
    position = body.position * PPM
    userData = body.userData
    name = userData["name"]

    bDraw = True
    if("visible" in userData):
        if(not userData["visible"]):
            bDraw = False

    if(name.startswith("reward")):
        if(bDraw):
            e = userData["energy"]
            width, color = 6, [80 - 80 * e, 80 * e, 0]
            if(name == "reward_small"):
                color = [10, 100, 255]
            draw_circle(screen, position, radius=int(r * PPM), color=color, width=0)
            width, color = 6, [255 - 255 * e,255 * e,0]
    elif(name == "epuck"):
        width, color = 4, [55, 100, 225]
    if(name == "ball"):
        width, color = 6, [10, 80, 0]
        draw_circle(screen, position, radius=int(r*PPM), color=color, width=0)
        width, color = 6, [10,120,0]

    if(bDraw):
        c, w = color, width
        if("color" in userData):
            c, w = userData["color"], 0
            draw_circle(screen, position, radius=int(r*PPM), color=c, width=w)
        draw_circle(screen, position, radius=int(r*PPM), color=color, width=width)

    if(name == "epuck"): drawEpuck(screen,r,body,color,width)
    elif(name == "wheel"): drawWheel(screen,r,body,color,width)


def my_draw_line(screen, points, color=(10, 80, 40, 10), width=1):
    vertices=[(X0+PPM * v[0], -Y0+SCREEN_HEIGHT - PPM * v[1]) for v in points]
    pygame.draw.line(screen, color, vertices[0], vertices[1], width)

def draw_contacts(screen, exp):
    for c in exp.obj.contacts:
        p = c.contact.worldManifold.points[0]
        n = p[0]*p[0]+p[1]*p[1]
        if(n > 0.01):
            p=(X0+PPM*p[0], -Y0+SCREEN_HEIGHT-PPM*p[1])     
            pygame.draw.circle(screen, (27,100,167,190), [int(p[0]),int(p[1])], int(6))

def draw_salient(screen, exp):
    salient = exp.getSalient()
    haptic = exp.haptic
    for i,p in enumerate(salient):
        pint = (X0+int(PPM*p[0]),-Y0+SCREEN_HEIGHT-int(PPM*p[1]))
        r = int(PPM*haptic[i])
        if(r > 50): r = 50
        rcolor = 30+5*r
        radius = 6
        if(rcolor > 255): rcolor = 255
        radiush = 2 + r/25
        if(radiush > 6): radiush = 6

        bDraw = True
        if(exp.name == "twooppositearms"):
            insideA = exp.boxA.fixtures[0].TestPoint(p)
            insideB = exp.boxB.fixtures[0].TestPoint(p)
            if(insideA or insideB): bDraw = False

        if(bDraw):
            pygame.draw.circle(screen, (10,27,100,190), pint, radius)
            pygame.draw.circle(screen, (rcolor,27,100,190), pint, radiush)



def draw_world(screen):
    for body in Box2DWorld.world.bodies:
        for fixture in body.fixtures:
            shape = fixture.shape
            if(body.active):
                if(isinstance(shape,Box2D.b2CircleShape)): box2d_draw_circle(screen, shape, body, fixture)
                if(isinstance(shape,Box2D.b2PolygonShape)): box2d_draw_polygon(screen, shape, body, fixture)
            else:
                if(isinstance(shape,Box2D.b2CircleShape)): box2d_draw_circle(screen, shape, body, fixture, color=(0,90,10), width=1)
                if(isinstance(shape,Box2D.b2PolygonShape)): box2d_draw_polygon(screen, shape, body, fixture, color=(0,90,10), width=1)


def draw_grid(screen):
    d = 10
    for i in range(-d,d):
        my_draw_line(screen,[[i,-d],[i,d]], color = (10,30,30,10))
        my_draw_line(screen,[[-d,i],[d,i]], color = (10,30,30,10))
