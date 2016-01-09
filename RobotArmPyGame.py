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

exp = Box2DWorld.ExpSetup(obj_type="box", salientMode = "minimum", debug = True, name ="human")
exp.setObjPos()
nao = exp.nao
obj = exp.obj

dm = np.array([1,1,1])

running=True
while running:
    # Check the event queue
    for event in pygame.event.get():
        if(event.type!=KEYDOWN): continue

        if(event.key== pygame.K_LEFT): nao.arms[0].deltaMotor(dm)
        if(event.key== pygame.K_RIGHT): nao.arms[0].deltaMotor(-dm)
        if(event.key== pygame.K_UP): nao.arms[1].deltaMotor(dm)
        if(event.key== pygame.K_DOWN): nao.arms[1].deltaMotor(-dm)

        if(event.key== pygame.K_SPACE): exp.setObjPos()

        if event.type==QUIT or event.key==K_ESCAPE:
            # The user closed the window or pressed escape
            running=False

    screen.fill((0,0,0,0))

    PyGameUtils.draw_contacts(screen,exp)
    PyGameUtils.draw_world(screen)
    PyGameUtils.my_draw_line(screen, exp.getObjLine() )
   
    Box2DWorld.step()
    exp.update()

    PyGameUtils.draw_salient(screen, exp)

    pygame.display.flip()              # Flip the screen and try to keep at the target FPS
    clock.tick(Box2DWorld.TARGET_FPS)
    pygame.display.set_caption("FPS: {:6.3}{}".format(clock.get_fps(), " "*5))
    
pygame.quit()
print('Done!')

