import sys
sys.path.append('./_utils/')
import pygame
import pygame.surfarray as surfarray
import PyGameUtils

import Box2DWorld 
from ExpRobotSetup import ExpSetupDualCartPole

box2dWH = (PyGameUtils.SCREEN_WIDTH, PyGameUtils.SCREEN_HEIGHT)

#***************************
#PYGAME initialization
#***************************
pygame.init()
screen = pygame.display.set_mode(box2dWH, 0, 32)
surfarray.use_arraytype('numpy')
pygame.display.set_caption('Arm Simulation Learning')
clock=pygame.time.Clock()

dm = 50
exp = ExpSetupDualCartPole(debug = True,xshift=-2.1)

running=True
while running:
    # Check the event queue
    for event in pygame.event.get():
        if(event.type!=pygame.KEYDOWN): continue

        if(event.key== pygame.K_LEFT): exp.setMotorSpeed(0,dm)
        if(event.key== pygame.K_RIGHT): exp.setMotorSpeed(0,-dm)
        if(event.key== pygame.K_UP): exp.setMotorSpeed(1,dm)
        if(event.key== pygame.K_DOWN): exp.setMotorSpeed(1,-dm)

        if(event.key== pygame.K_SPACE): exp.resetPosition()

        if event.type==pygame.QUIT or event.key==pygame.K_ESCAPE:
            # The user closed the window or pressed escape
            running=False

    screen.fill((0,0,0,0))

    #PyGameUtils.draw_contacts(screen,exp)
    PyGameUtils.draw_world(screen)
    
    Box2DWorld.step()
    exp.update()

    PyGameUtils.draw_salient(screen, exp)

    #PyGameUtils.my_draw_line(screen,[exp.getSalient()[0],exp.getLinkExtreme(0)])
    #PyGameUtils.my_draw_line(screen,[exp.getSalient()[1],exp.getLinkExtreme(1)])

    pygame.display.flip()              # Flip the screen and try to keep at the target FPS
    clock.tick(Box2DWorld.TARGET_FPS)
    pygame.display.set_caption("FPS: {:6.3}{}".format(clock.get_fps(), " "*5))
    
pygame.quit()
print('Done!')

