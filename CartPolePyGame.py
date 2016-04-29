import pygame
from pygame.locals import *
import pygame.surfarray as surfarray
import numpy as np
import sys
sys.path.append('./_utils/')
import PyGameUtils
import Box2DWorld

box2dWH = (PyGameUtils.SCREEN_WIDTH, PyGameUtils.SCREEN_HEIGHT)

#***************************
#PYGAME initialization
#***************************
pygame.init()
screen = pygame.display.set_mode(box2dWH, 0, 32)
surfarray.use_arraytype('numpy')
pygame.display.set_caption('Arm Simulation Learning')
clock=pygame.time.Clock()

dm = 10
exp = Box2DWorld.ExpSetupDualCartPole(debug = True)

running=True
while running:
    # Check the event queue
    for event in pygame.event.get():
        if(event.type!=KEYDOWN): continue

        if(event.key== pygame.K_LEFT): exp.cart.deltaMotor(dm)
        if(event.key== pygame.K_RIGHT): exp.cart.deltaMotor(-dm)

        #if(event.key== pygame.K_SPACE): exp.setObjPos()

        if event.type==QUIT or event.key==K_ESCAPE:
            # The user closed the window or pressed escape
            running=False

    screen.fill((0,0,0,0))

    #PyGameUtils.draw_contacts(screen,exp)
    PyGameUtils.draw_world(screen)
    
    Box2DWorld.step()
    #exp.update()

    #PyGameUtils.draw_salient(screen, exp)

    pygame.display.flip()              # Flip the screen and try to keep at the target FPS
    clock.tick(Box2DWorld.TARGET_FPS)
    pygame.display.set_caption("FPS: {:6.3}{}".format(clock.get_fps(), " "*5))
    
pygame.quit()
print('Done!')

