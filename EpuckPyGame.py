import sys
sys.path.append('./_utils/')
import numpy as np
import pygame
import pygame.surfarray as surfarray
from pygame.locals import *


import PyGameUtils
import Box2DWorld 
from ExpRobotSetup import ExpSetupEpuck

box2dWH = (PyGameUtils.SCREEN_WIDTH, PyGameUtils.SCREEN_HEIGHT)

#***************************
#PYGAME initialization
#***************************
pygame.init()
PyGameUtils.setScreenSize(640,480)
box2dWH = (PyGameUtils.SCREEN_WIDTH, PyGameUtils.SCREEN_HEIGHT)

#flags = FULLSCREEN | DOUBLEBUF
#screen = pygame.display.set_mode(box2dWH, flags, 8)
screen = pygame.display.set_mode(box2dWH, 0, 32)
screen.set_alpha(None)
surfarray.use_arraytype('numpy')

pygame.display.set_caption('Epuck Simulation')
clock=pygame.time.Clock()

exp = ExpSetupEpuck(n=2, debug = True)

running=True
while running:
    # Check the event queue
    for event in pygame.event.get():
        if(event.type!=pygame.KEYDOWN): continue

        if(event.key== pygame.K_LEFT): exp.setMotors(motors=[-1,1])
        if(event.key== pygame.K_RIGHT): exp.setMotors(motors=[1,-1])
        if(event.key== pygame.K_UP): exp.setMotors(motors=[1,1])
        if(event.key== pygame.K_DOWN): exp.setMotors(motors=[-1,-1])
        if(event.key== pygame.K_SPACE): exp.setMotors(motors=[0,0])

        if event.type==pygame.QUIT or event.key==pygame.K_ESCAPE:
            # The user closed the window or pressed escape
            running=False

    screen.fill((0,0,0,0))

    #PyGameUtils.draw_contacts(screen,exp)
    PyGameUtils.draw_world(screen)
   
    exp.update()
    Box2DWorld.step()

    #PyGameUtils.draw_salient(screen, exp)

    pygame.display.flip()              # Flip the screen and try to keep at the target FPS
    clock.tick(Box2DWorld.TARGET_FPS)
    pygame.display.set_caption("FPS: {:6.3}{}".format(clock.get_fps(), " "*5))
    
pygame.quit()
print('Done!')

