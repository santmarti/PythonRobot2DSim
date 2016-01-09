import pygame
import pygame.surfarray as surfarray

import numpy as np
import Box2D # The main library
from Box2D.b2 import * # This maps Box2D.b2Vec2 to vec2 (and so on)
import Box2DWorld
#from OSC import OSCServer

SCREEN_WIDTH, SCREEN_HEIGHT=640,480
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
# PyGame Drawing function utils         
# ****************************************************************************     

        
def draw_polygon(screen, vertices, color = (0,255,0), width=3):
    vertices=[(X0+v[0], -Y0+SCREEN_HEIGHT-v[1]) for v in vertices]
    pygame.draw.polygon(screen, color, vertices, width)
        
def box2d_draw_polygon(screen, polygon, body, fixture, color = [], width=3):
    if(len(color)==0): color = colors[body.type]
    vertices=[(body.transform*v)*PPM for v in polygon.vertices]
    draw_polygon(screen, vertices, color, width)

def draw_circle(screen, position=(0,0), radius = 1, color=(27,200,7,190), width=0):
    global X0,Y0
    position=(X0+position[0], -Y0+SCREEN_HEIGHT-position[1])
    pygame.draw.circle(screen, color, [int(x) for x in position], radius, width)

def box2d_draw_mycircle(screen, pos, radius, color):
    draw_circle(screen, (pos[0]*PPM,pos[1]*PPM), radius=int(radius*PPM), color=color)

def box2d_draw_circle(screen, circle, body, fixture, color=[], width=3):
    if(len(color)==0): color = colors[body.type]
    position=body.transform*circle.pos*PPM
    draw_circle(screen, position, radius=int(circle.radius*PPM), color=color, width=width)


def my_draw_line(screen, points, color = (10,80,40,10)):
    vertices=[(X0+PPM * v[0], -Y0+SCREEN_HEIGHT - PPM * v[1]) for v in points]
    pygame.draw.line(screen, color, vertices[0],vertices[1])

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
