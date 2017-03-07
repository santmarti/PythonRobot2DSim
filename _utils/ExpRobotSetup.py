import numpy as np
import Box2D
from Box2DWorld import (world, arm, createBox, createCircle, createTri, createRope,
                        myCreateRevoluteJoint, myCreateDistanceJoint,
                        myCreateLinearJoint, collisions, RayCastCallback)

from VectorFigUtils import vnorm, dist
from Robots import NaoRobot, CartPole, Epuck
import random



def addWalls(pos, dx=3, dh=0, h=2.8, th=0, bHoriz=True, bVert=True, damping = 0):               
    x, y = pos
    wl = 0.2
    yh = (5 + 1) / 2.0
    if(bHoriz):
        createBox((x, y - 1 - dh + th), w=h + dh + wl + th, h=wl, bDynamic=False, damping=damping, name="wall_top")
        createBox((x, y + 5 + dh + th), w=h + dh + wl + th, h=wl, bDynamic=False, damping=damping, name="wall_bottom")
    if(bVert):
        createBox((x - dx - wl, y + yh - 1 + dh / 2 + th), w=wl, h=h + dh, bDynamic=False, damping=damping, name="wall_left")
        createBox((x + dx + wl, y + yh - 1 + dh / 2 + th), w=wl, h=h + dh, bDynamic=False, damping=damping, name="wall_right")

def addReward(who, pos=(0,0), vel=(0,0), reward_type=0, bDynamic=True, bCollideNoOne=False):
    if(reward_type == 0):
        name, r = "reward", 0.27
    else:
        name, r = "reward_small", 0.2

    obj = createCircle(position=pos, bDynamic=bDynamic, bCollideNoOne=bCollideNoOne, density=5, damping=0, friction=0, name=name, r=r)
    obj.userData["energy"] = 1.0
    obj.userData["visible"] = 1.0
    obj.linearVelocity = vel
    who.objs.append(obj)


# *****************************************************************
# Experimental Setup Randall 1D Agent
# *****************************************************************


class ExpSetupRandall():
    """Experimental setup including 2 agents in a 1D horizontal line."""

    def __init__(self, n=2, radius=0.2, frontIR=12, debug=False):
        global bDebug
        bDebug = debug
        print "-------------------------------------------------"
        self.yini = -1.2
        self.radius = radius
        world.gravity = Box2D.b2Vec2(0, -1.01)
        self.epucks = [Epuck(position=[-1 + 2 * i, self.yini], frontIR=frontIR, bHorizontal=True) for i in range(n)]

        for e in self.epucks:
            e.userData["score"] = 0
            e.userData["reward"] = 0

        self.walls_dx = 7.25
        addWalls((0, 0), dx=self.walls_dx, dh=2, bHoriz=False)
        createBox((0, -2), w=7.65, h=0.2, bDynamic=False, name="floor")

        self.callback = RayCastCallback()

        self.objs = []

        self.box = None
        # self.setOcclusion()
        self.phase_names = ["Training", "Easy", "Intermmediate", "Difficult"]

    def setOcclusion(self, y=3, h=1):
        """Create an occulsion box that has no collisions."""
        self.box = createBox([0, y], w=self.walls_dx, h=h, bDynamic=False, bCollideNoOne=True, name="occlusion") 
        self.box.userData["visible"] = 1.0
        self.box.userData["height"] = h
        self.box.userData["y"] = y

    def clearOcclusion(self):
        """Clears the occulsion box."""
        if(self.box is not None):
            world.DestroyBody(self.box)
        self.box = None

    def update(self):
        epucks = self.epucks
        for e in epucks:
            e.update()
            x, y = e.body.position
            e.body.position = [x, self.yini]

    def setVelocity(self, epuck=0, vel=[0,0]):
        self.epucks[epuck].body.linearVelocity = vel

    def setMotor(self, epuck=0, motor=1):
        self.epucks[epuck].motors = [motor, 0]

    def action(self, epuck=0, action=0):
        if(action == 0):
            motors = [-1, 0]
        elif(action == 1):
            motors = [1, 0]
        elif(action == 2):
            motors = [0, 0]
        self.epucks[epuck].motors = motors

    def getIRs(self, epuck=0):
        return self.epucks[epuck].getIRs()


# *****************************************************************
# Experimental Setup Epuck
# *****************************************************************

class ExpSetupEpuck(object):
    """Exp setup class with two epucks and two reward sites."""

    def __init__(self, n=1, debug=False):
        """Create the two epucks, two rewards and walls."""
        global bDebug
        bDebug = debug
        print "-------------------------------------------------"
        th = .2
        positions = [(-3, 2 + th), (3, 2 + th)]
        angles = [2 * np.pi, np.pi]
        self.epucks = [Epuck(position=positions[i], angle=angles[i], nother=2, nrewsensors=4) for i in range(n)]
        addWalls((0, 0), dx=3.75, dh=0.1, h=3, th=th)
        self.objs = []
        addReward(self, pos=(0, 4 + th), vel=(0, 0), bDynamic=False, bCollideNoOne=True)
        addReward(self, pos=(0, 0 + th), vel=(0, 0), reward_type=1, bDynamic=False, bCollideNoOne=True)

    def update(self):
        """Update of epucks positions and gradient sensors: other and reward."""
        for e in self.epucks:
            e.update()
            pos = e.getPosition()

            for g in e.GradSensors:
                if(g.name == "other"):
                    centers = [o.getPosition() for o in self.epucks if o != e]
                    g.update(pos, e.getAngle(), centers)
                elif(g.name == "reward"):
                    centers = [o.position for o in self.objs[:1]]
                    g.update(pos, e.getAngle(), centers)
                    centers = [o.position for o in self.objs[-1:]]
                    g.update(pos, e.getAngle(), centers, extremes=1)



    def setMotors(self, epuck=0, motors=[10, 10]):
        self.epucks[epuck].motors = motors


# *****************************************************************
# Experimental Setup Class : Dual CartPole holding object
# *****************************************************************

class ExpSetupDualCartPole:

    max_motor_speed = 30

    def __init__(self, xshift=0, salientMode="center", name="simple", debug = False, objBetween = 4, objWidth = 0.1, objForce=100, bSelfCollisions=True):
        global world, bDebug
        bDebug = debug
        print "-------------------------------------------------"
        print "Created Exp Dual Cart Pole Setup ", name, "Debug: ", bDebug

        world.gravity = Box2D.b2Vec2(0, -140)

        self.name = name
        self.salient = []
        self.haptic = [0.1] * 10
        self.addWalls(pos=[0, 0])
        self.xshift = xshift
        self.objBetween = objBetween
        xpos = 1.5

        self.carts = [CartPole(name="cartLeft", position=(-xpos + xshift, -0.3), bHand=1, d=0.8, collisionGroup=1), CartPole(name="cartRight",position=(xpos+xshift,-0.3),bHand=2,d=0.8,collisionGroup = 2)]

        if(objWidth > 0):
            bodyleft, bodyright = self.carts[0].box, self.carts[1].box
            if(objBetween == 1):
                self.link = [createBox((xshift, 2), xpos * 0.8, objWidth, bDynamic=True, restitution=0.8)]
            elif(objBetween >= 2 and objBetween <= 3):
                y = 1.53
                if(objBetween == 3):
                    objLong = xpos * 0.8 / 3.5
                else:
                    objLong = xpos * 0.8 / 2.0
                bodyA = createBox((xshift - objLong, y), objLong, objWidth, bDynamic=True, restitution = 0.8)
                bodyB = createBox((xshift + objLong, y), objLong, objWidth - 0.03, bDynamic=True, restitution = 0.8)
                self.joint = myCreateLinearJoint(bodyA, bodyB, force=objForce, lowerTranslation = -0.9,upperTranslation = 0)
                self.link = [bodyA, bodyB]
                if(objBetween == 3):
                    dy = 0.3
                    bodyA.position = (xshift - objLong, y - dy)
                    bodyB.position = (xshift + objLong, y - dy)
                    self.jointleft = myCreateRevoluteJoint(bodyleft,bodyA,(xshift-1.95*objLong,y+objWidth/2.0-dy),lowerAngle = -2*np.pi, upperAngle = 2*np.pi)
                    self.jointright = myCreateRevoluteJoint(bodyright,bodyB, (xshift+1.95*objLong,y+objWidth/2.0-dy),lowerAngle = -2*np.pi, upperAngle = 2*np.pi)
            elif(objBetween == 4):
                pini = (bodyleft.position[0] + 0.8, bodyleft.position[1])
                rbodies, rlinks = createRope(pini, 10, r=0.1, density=0.1)
                self.link = rbodies
                myCreateDistanceJoint(bodyleft, rbodies[0], dx=0.8)
                myCreateDistanceJoint(bodyright, rbodies[-1], dx=-0.8)

        if(bSelfCollisions):
            collisionGroup = None
        else:
            collisionGroup = -1

        if(bDebug):
            print "Exp Setup created", "salient points: ", self.salient

    def update(self):
        for i in [0, 1]:
            self.carts[i].update()

    def addWalls(self, pos):
        """Limits of the world."""
        x, y = pos
        wl = 0.2
        h = (5 + 1) / 2.0
        l = 6
        createBox((x, y - 1), w=l + 2 * wl, h=wl, bDynamic=False)
        # createBox((x,y+5), w = 3, h = wl, bDynamic=False)
        createBox((x - l - wl, y + h - 1), w=wl, h=2.8, bDynamic=False)
        createBox((x + l + wl, y + h - 1), w=wl, h=2.8, bDynamic=False)

    def getSalient(self):
        return [cart.getBodyPos() for cart in self.carts]

    def getLinkExtreme(self,i):
        if(i == 0):
            body = self.link[0]
            if(body is Box2D.b2CircleShape):
                pos = body.center
                return
            shape = body.fixtures[0].shape
            vertices = [body.transform * v for v in shape.vertices]
            pos = (vertices[0] + vertices[3]) / 2
        if(i == 1):
            body = self.link[-1]
            if(body is Box2D.b2CircleShape):
                pos = body.center
                return
            shape = body.fixtures[0].shape
            vertices = [body.transform * v for v in shape.vertices]
            pos = (vertices[1] + vertices[2]) / 2
        return pos

    def getLinkDistance(self, i):
        return dist(self.getSalient()[i], self.getLinkExtreme(i))

    def setMotorSpeed(self, i, speed):
        self.carts[i].setMotorSpeed(speed)

    def resetPositionBody(self, b):
        b.linearVelocity = [0, 0]
        b.angularVelocity = 0
        b.angle = 0
        b.position = (self.xshift, 1.5)

    def resetPosition(self):
        bcartleft, bcartright = self.carts[0].box, self.carts[1].box
        for i in [0, 1]:
            self.carts[i].resetPosition()

        if(self.objBetween < 4):
            for b in self.link:
                self.resetPositionBody(b)
        else:
            for i in range(len(self.link) - 2):
                c = self.link[i + 1]
                c.position = (bcartleft.position[0] + 0.8 + 0.1 * (i + 1), bcartleft.position[1])


    def getIRs(self):
        return [cart.getIR() for cart in self.carts]

    def getAngles(self):
        return [cart.getAngle() for cart in self.carts]

    def getPositions(self):
        return [cart.getPosition() for cart in self.carts]

    def getVelocities(self):
        return [cart.getVelocity()[0] for cart in self.carts]

    def getLinkPos(self):
        if(len(self.link) == 1):
            return self.link.position
        else:
            return (self.link[0].position + self.link[-1].position) / 2.0



# *****************************************************************
# Experimental Setup Class : NaoRobot class plus walls plus object
# *****************************************************************


class ExpSetupNao:
    max_motor_speed = 30

    def __init__(self, pos_obj = (0,1.3), pos_nao = (0,0), obj_type = "circle", salientMode = "center", name="bimanual", debug = False, bTwoArms=True, bSelfCollisions=True):
        global bDebug
        bDebug = debug
        print "-------------------------------------------------------------"
        print "Created Exp Bimanual Setup: ", name, "Debug: ", bDebug, "Object"
        self.name = name.lower()
        self.dm_lim = 1
        self.v_lim = 0.3

        self.salient = []
        self.haptic = []
        self.obj_type = obj_type

        if(bSelfCollisions): collisionGroup=None
        else: collisionGroup=-1

        self.name_robot = "simple"
        bOppositeArms = False
        self.objs = []

        if(self.name == "bimanual"):
            self.name_robot = "human" 
            addWalls(pos_nao)
            self.iniThreeObjects(pos_obj, obj_type)
        elif(self.name == "twooppositearms"):
            self.name_robot = "human"
            bOppositeArms = True
            self.obj_type = "circle"
            self.iniConstrainedObject(pos_obj)
            w = 0.3
            self.boxA = createBox([-0.8, 1.5], w=w, h=w, bDynamic=False, bCollideNoOne=True, name="boxA") 
            self.boxB = createBox([0.8, 1.5], w=w, h=w, bDynamic=False, bCollideNoOne=True, name="boxB") 

        self.nao = NaoRobot(pos_nao, name=self.name_robot, bTwoArms=bTwoArms, bOppositeArms=bOppositeArms, collisionGroup=collisionGroup)
        self.arms = self.nao.arms

        self.ini_obj_pos = pos_obj

        self.changeSalientMode(salientMode)
        if(bDebug):
            print "Exp Setup created", self.salientMode, "salient points: ", self.salient 


    def iniThreeObjects(self,pos_obj,obj_type):
        self.objs.append(createCircle(pos_obj, r=0.45))
        self.objs.append(createTri(pos_obj, r=0.45))
        self.objs.append(createBox(pos_obj, wdiv=1, hdiv=1, w=0.35, h=0.35))

        self.target_objs = []
        self.target_objs.append(createCircle(pos_obj, r=0.45))
        self.target_objs.append(createTri(pos_obj, r=0.45))
        self.target_objs.append(createBox(pos_obj, wdiv=1, hdiv=1, w=0.35, h=0.35))
        for t in self.target_objs:
            t.active = False
            t.position = [2, 4]

        x = -2
        for o in self.objs:
            o.position, x = (x, 4), x + 1

        if(obj_type == "circle"):
            self.obj, self.target_obj = self.objs[0], self.target_objs[0]
        elif(obj_type == "box"):
            self.obj, self.target_obj = self.objs[2], self.target_objs[2]


    def iniConstrainedObject(self,pos_obj):
        obj = createCircle(pos_obj, r=0.3, density=0.01, name="ball")
        self.objs.append(obj)
        obj.position = [0, 1.5]
        self.obj = self.objs[0]
        obj.userData["name"] = "toy"

        bar = createBox(obj.position, w=1, h=0.001, bDynamic=False, bCollideNoOne=True)
        bar.userData["name"] = "bar"

        self.joint = myCreateLinearJoint(bar, obj, force=0, lowerTranslation=-0.82, upperTranslation=0.82)

    def setTargetObj(self, pos, angle=0):
        self.target_obj.position = pos
        self.target_obj.angle = angle

    def changeSalientMode(self,salientMode):
        self.salientMode = salientMode
        self.update()

    def start(self, obj_pos=[]):
        if(obj_pos == []):
            self.obj.position = self.ini_obj_pos
        else:
            self.obj.position = obj_pos
        self.nao.restPosition(online=False)

    def getSalient(self):
        return self.salient

    def getSalientType(self, i):
        self.update()
        narms = len(self.nao.arms)
        leftsalient = len(self.nao.arms[0].salient)
        if(i < leftsalient):
            return "left"
        if(narms > 1 and i < 2 * leftsalient):
            return "right"
        return "obj"

    def updateSalient(self):
        naosalient = self.nao.getSalient()
        self.salient = [s for s in naosalient]

        if(self.obj_type not in ["box", "circle"]):
            return

        b = self.obj.position
        b = [round(e, 2) for e in b]
        body = self.obj
        shape = self.obj.fixtures[-1].shape

        if(self.salientMode == "center"):
            self.salient += [(b[0], b[1])]
        elif(self.salientMode == "laterals"):
            if(self.obj_type == "circle"):
                self.salient += [(b[0], b[1])]
                self.salient += [(b[0] - shape.radius,b[1])]
                self.salient += [(b[0] + shape.radius,b[1])]
            if(self.obj_type == "box"):
                v = [body.transform * v for v in shape.vertices]
                self.salient += [(b[0],b[1])]
                self.salient += [(v[1] + v[2])/2.0]
                self.salient += [(v[0] + v[3])/2.0]
        elif(self.salientMode == "minimum"):
            self.salient += [(b[0], b[1])]
            if(self.obj_type == "box"):
                v = [body.transform * v for v in shape.vertices]
                self.salient += [[v[0][0], v[0][1]]]
        elif(self.salientMode == "all"):
            self.salient += [(b[0], b[1])]
            if(self.obj_type == "box"):
                v = [body.transform * v for v in shape.vertices]
                for p in v:
                    self.salient += [p]
                self.salient += [(v[1] + v[2]) / 2.0]
                self.salient += [(v[0] + v[3]) / 2.0]
                self.salient += [(v[2] + v[3]) / 2.0]
                self.salient += [(v[0] + v[1]) / 2.0]

    def getFinalHaptic(self, arm=0):
        """First salient is always the end point."""
        if(arm == 0):
            i = 0
        else:
            i = len(self.nao.arms[0].salient)
        return self.haptic[i]

    def updateHaptic(self):
        if(len(self.haptic) < len(self.salient)):
            self.haptic = [0] * len(self.salient)
        maxd = 0.5
        dt = 1.3
        for i, s in enumerate(self.salient):
            sbox = (s[0], s[1])
            mind = maxd
            if(self.haptic[i] > 0):
                self.haptic[i] /= dt
            else:
                self.haptic[i] = 0
            for c in self.obj.contacts:
                cpos = c.contact.worldManifold.points[0]
                if(vnorm(cpos) < 0.01):
                    continue
                d = dist(cpos, sbox)
                if(d < maxd and d < mind):
                    mind = d
                    h = 1 - d / maxd
                    self.haptic[i] = h

    def update(self, iarm=-1):
        err = self.nao.update(iarm=arm)
        self.updateSalient()
        self.updateHaptic()
        return err

    def getObjLine(self):
        b = self.obj.position
        b = (b[0], b[1])
        n = self.nao.body_pos
        return [n, b]

    def setObjPos(self, p=[], angle=0):
        if(len(p) == 0):
            p = self.ini_obj_pos
        self.obj.position = p
        self.obj.angle = angle
        for i in range(len(self.haptic)):
            self.haptic[i] = 0
        self.update()

    def getObjPos(self, bAngle=False, PPM=1):
        self.update()
        p = self.obj.position
        p = [PPM * round(p[0], 2), PPM * round(p[1], 2)]
        if(bAngle):
            return p, self.obj.angle
        else:
            return p

    def getMotorHistory(self, iarm=0, t=-1):
        """One iteration before by default."""
        arms = self.nao.arms
        h = arms[iarm].history
        if(len(h) >= abs(t)):
            return h[t]
        else:
            return [0] * arms[iarm].nparts

    def deltaMotor(self, dm):
        return self.nao.deltaMotor(dm)


    def resetOpposite(self):
        collisions(False)
        da = self.nao.m_maxs()[0] / 4
        db = self.nao.m_mins()[1] / 4
        self.nao.gotoTargetJoints([da, -db] + [0] * (self.nao.nparts - 2), iarm=0)
        self.nao.gotoTargetJoints([da, -db] + [0] * (self.nao.nparts - 2), iarm=1)
        collisions(True)
        self.setObjPos()






