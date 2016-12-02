from Box2DWorld import createBox
from Arm import Arm
           
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

    def __init__(self, position=(0,0), name="simple", bMatplotlib = False, d = 1, bHand = 0, collisionGroup=None):
        global bDebug
        self.name = name
        self.ini_pos = position
        self.salientMode = "all"
        self.circle = createCircle(position, r=d*0.6, dynamic=True, bMatplotlib = True)
        self.box = createBox( (position[0],position[1]+d*1.9), d*0.2, d*2, dynamic=True)
        self.joint = myCreateRevoluteJoint(self.circle,self.box,position,iswheel=True)    
        self.bHand = bHand

        if(bHand>0): 
            body = self.box
            h = d*0.15
            w = d*0.4
            pos = (2*w-d*0.2,0)
            if(bHand == 2): pos = (-2*w+d*0.2,0)
            fixtureDef = createBoxFixture(body, pos, width=w, height=h, collisionGroup = collisionGroup)
            fixture = body.CreateFixture(fixtureDef)

        self.motor_speed = 0
        self.angle = 0

    def resetPosition(self):
        ipos = self.ini_pos
        self.motor_speed = 0
        self.angle = 0
        self.joint.motorSpeed = 0
        self.joint.torque = 0
        self.box.linearVelocity = [0,0]
        self.box.angularVelocity = 0
        self.box.angle = 0
        self.box.position = (ipos[0],ipos[1]+1.9)

        self.circle.linearVelocity = [0,0]
        self.circle.angularVelocity = 0
        self.circle.angle = 0
        self.circle.position = (ipos[0],ipos[1])

    def getBodyPos(self):
        body = self.box
        shape = body.fixtures[1].shape
        verticesHand=[body.transform*v for v in shape.vertices]

        shape = body.fixtures[0].shape
        vertices=[body.transform*v for v in shape.vertices]

        if(self.bHand == 2): 
            d = vertices[2] - vertices[1]
            d = (d[0]/12.,d[1]/12.)
            p = (verticesHand[1] + verticesHand[2])/2.0 + d 
        else: 
            d = vertices[2] - vertices[1]
            d = (d[0]/12.,d[1]/12.)
            p = (verticesHand[3] + verticesHand[0])/2.0 + d

        ret = [p[0], p[1]]
        ret = [round(e,2) for e in ret]
        return ret


    def setMotorSpeed(self,speed):  
        self.joint.motorSpeed = speed

    def getAngle(self):
        return self.box.angle

    def getPosition(self):
        return self.box.position[0]

    def getVelocity(self):
        return self.box.linearVelocity

    def update(self):
        self.setMotorSpeed( self.motor_speed )
        self.angle = self.getAngle()



# *****************************************************************
# Experimental Setup Class : Dual CartPole holding object
# *****************************************************************

class ExpSetupDualCartPole:

    max_motor_speed = 30

    def __init__(self, xshift=0, salientMode = "center", name="simple", debug = False, objBetween = 1, objWidth = 0.1, objForce=100, bSelfCollisions=True):
        global world, bDebug
        bDebug = debug        
        print "-------------------------------------------------"
        print "Created Exp Dual Cart Pole Setup ", name, "Debug: ", bDebug

        #world = Box2D.b2World(gravity=[0.0, -10])
        world.gravity = Box2D.b2Vec2(0,-1000) 

        self.name = name
        self.salient = []
        self.haptic = [0.1]*10
        self.addWalls([0,0])
        self.xshift = xshift
        
        xpos = 1.5
        self.carts = [CartPole(position=(-xpos+xshift,0),bHand=1,d=0.8), CartPole(position=(xpos+xshift,0),bHand=2,d=0.8)]
        
        if(objWidth > 0): 
            if(objBetween == 2): 
                self.link = [createBox( (xshift,2), xpos*0.8, objWidth, dynamic=True, restitution = 0.8)]
            elif(objBetween == 1): 
                objLong = xpos*0.8 / 2.0
                bodyA = createBox( (xshift-objLong,2), objLong, objWidth, dynamic=True, restitution = 0.8)
                bodyB = createBox( (xshift+objLong,2), objLong, objWidth-0.03, dynamic=True, restitution = 0.8)
                self.joint = myCreateLinearJoint(bodyA,bodyB,force=objForce)
                self.link = [bodyA,bodyB]

        if(bSelfCollisions): collisionGroup=None
        else: collisionGroup=-1
      
        if(bDebug):
            print "Exp Setup created", "salient points: ", self.salient 


    def addWalls(self,pos, bMatplotlib=False): # WALL LIMITS of the world               
        x,y = pos 
        wl = 0.2
        h = (5+1)/2.0
        l = 4.45
        createBox((x,y-1), w = l+2*wl, h = wl, dynamic=False, bMatplotlib = bMatplotlib)
        #createBox((x,y+5), w = 3, h = wl, dynamic=False, bMatplotlib = bMatplotlib)
        createBox((x-l-wl,y+h-1), w = wl, h = 2.8, dynamic=False, bMatplotlib = bMatplotlib)
        createBox((x+l+wl,y+h-1), w = wl, h = 2.8, dynamic=False, bMatplotlib = bMatplotlib)

    def getSalient(self):
        return [cart.getBodyPos() for cart in self.carts]

    def getLinkExtreme(self,i):
        if(i==0):
            body = self.link[0]
            shape = body.fixtures[0].shape
            vertices=[body.transform*v for v in shape.vertices]
            pos=(vertices[0]+vertices[3])/2
        if(i==1):
            body = self.link[-1]
            shape = body.fixtures[0].shape
            vertices=[body.transform*v for v in shape.vertices]
            pos=(vertices[1]+vertices[2])/2
        return pos

    def getLinkDistance(self,i):
        return dist(self.getSalient()[i],self.getLinkExtreme(i))

    def setMotorSpeed(self,i,speed):
        self.carts[i].setMotorSpeed(speed)

    def resetPositionBody(self,b):
        b.linearVelocity = [0,0]
        b.angularVelocity = 0
        b.angle = 0
        b.position = (self.xshift,2)
        for i in [0,1]:
            self.carts[i].resetPosition()

    def resetPosition(self):
        for b in self.link:
            self.resetPositionBody(b)

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
            return (self.link[0].position + self.link[-1].position)/2.0




