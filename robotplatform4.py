# -*- coding: utf-8 -*-
"""
Created on Sun Jun 10 18:28:23 2018

@author: Richard Bloemenkamp
"""
import pybullet as p
import time
import numpy as np
    
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW ,0)

p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0,0)


#Plat robot part shapes
sh_body = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.10, 0.08, 0.02])
sh_extraweight = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.10, 0.08, 0.02])
sh_leg = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.005, 0.005, 0.30])
sh_steer = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01, 0.01, 0.02])
sh_wheel = p.createCollisionShape(p.GEOM_CYLINDER,radius=0.03, height=0.03)
shv_wheel = p.createVisualShape(p.GEOM_CYLINDER,radius=0.03, length=0.03, rgbaColor=[0,0,0,1])

#sh_roll = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.02, 0.02, 0.02])
#sh_hip = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.02, 0.02, 0.02])
#sh_knee = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.02, 0.02, 0.02])
#sh_foot = p.createCollisionShape(p.GEOM_SPHERE,radius=0.04)

#The platform robot is the body with other shapes linked to it
body_Mass = .1
visualShapeId = -1 
link_Masses=[.1, .1, .1,
             .1, .1, .1,
             .1, .1, .1,
             .1, .1, .1,
             2]
linkCollisionShapeIndices=[sh_leg, sh_steer, sh_wheel,
                           sh_leg, sh_steer, sh_wheel,
                           sh_leg, sh_steer, sh_wheel,
                           sh_leg, sh_steer, sh_wheel,
                           sh_extraweight]
nlnk=len(link_Masses)
#linkVisualShapeIndices=[-1]*nlnk    #=[-1,-1,-1, ... , -1]
linkVisualShapeIndices=[-1, -1, shv_wheel,
                        -1, -1, shv_wheel, 
                        -1, -1, shv_wheel,
                        -1, -1, shv_wheel,
                        -1]
#link positions wrt the link they are attached to
xhipf=0.1
xhipb=-0.1
yhipl=0.1

xoffh=0.00
yoffh=0.00
hu=0.35
hl=0.05
linkPositions=[[xhipf, yhipl, 0], [0, 0, -hu],[0, 0, -hl],
               [xhipf, -yhipl, 0], [0, 0, -hu],[0, 0, -hl],
               [xhipb, yhipl, 0], [0, 0, -hu],[0, 0, -hl],
               [xhipb, -yhipl, 0], [0, 0, -hu],[0, 0, -hl],
               [0,0,+0.1]]
linkOrientations=[[0,0,0,1]]*nlnk
linkOrientations[2]=[1,0,0,1]
linkOrientations[5]=[1,0,0,1]
linkOrientations[8]=[1,0,0,1]
linkOrientations[11]=[1,0,0,1]

linkInertialFramePositions=[[0,0,0]]*nlnk
#Note the orientations are given in quaternions (4 params). There are function to convert of Euler angles and back
linkInertialFrameOrientations=[[0,0,0,1]]*nlnk
#indices determine for each link which other link it is attached to
# for example 3rd index = 2 means that the front left knee jjoint is attached to the front left hip
indices=[0, 1, 2,
         0, 4, 5,
         0, 7, 8,
         0,10,11,
         0]
#Most joint are revolving. The prismatic joints are kept fixed for now
jointTypes=[p.JOINT_PRISMATIC, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, 
            p.JOINT_PRISMATIC, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE,
            p.JOINT_PRISMATIC, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE,
            p.JOINT_PRISMATIC, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE,
            p.JOINT_PRISMATIC]
#revolution axis for each revolving joint
axis=[[0,0,1], [0,0,1], [0,0,1],
      [0,0,1], [0,0,1], [0,0,1],
      [0,0,1], [0,0,1], [0,0,1],
      [0,0,1], [0,0,1], [0,0,1],
      [0,0,1]]

#Drop the body in the scene at the following body coordinates
basePosition = [0,0,1]
baseOrientation = [0,0,0,1]
#Main function that creates the platform
platform = p.createMultiBody(body_Mass,sh_body,visualShapeId,basePosition,baseOrientation,
                        linkMasses=link_Masses,
                        linkCollisionShapeIndices=linkCollisionShapeIndices,
                        linkVisualShapeIndices=linkVisualShapeIndices,
                        linkPositions=linkPositions,
                        linkOrientations=linkOrientations,
                        linkInertialFramePositions=linkInertialFramePositions,
                        linkInertialFrameOrientations=linkInertialFrameOrientations,
                        linkParentIndices=indices,
                        linkJointTypes=jointTypes,
                        linkJointAxis=axis)			
##Due to the weight the prismatic extraweight block needs to be motored up
#joint=16
#p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=0.01,force=1000,maxVelocity=300)
##Same for the prismatic feet spheres
#joint=3
#p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=300)
#joint=7
#p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=300)
#joint=11
#p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=300)
#joint=15
#p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=300)

#fix extraweight
joint=12
p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=0.0,force=1000,maxVelocity=1)



#Scenery e.g. an inclined box
boxHalfLength = 2.5
boxHalfWidth = 2.5
boxHalfHeight = 0.2
sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
#mass = 1
#block=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
#                        basePosition = [-2,0,-0.1],baseOrientation=[0.0,0.1,0.0,1])
sth=0.25
block2=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [2.75,0,-0.2+1*sth],baseOrientation=[0.0,0.0,0.0,1])
block3=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [2.75+0.33,0,-0.2+2*sth],baseOrientation=[0.0,0.0,0.0,1])
block4=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [2.75+0.66,0,-0.2+3*sth],baseOrientation=[0.0,0.0,0.0,1])
block5=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [2.75+0.99,0,-0.2+4*sth],baseOrientation=[0.0,0.0,0.0,1])



def waitsec(sec):
    t0=time.time()
    t=time.time()
    while ((t-t0)<sec):
        t=time.time()



#Add earth like gravity
p.setGravity(0,0,-9.81)
p.setRealTimeSimulation(1)
#Point the camera at the robot at the desired angle and distance
p.resetDebugVisualizerCamera( cameraDistance=2, cameraYaw=-0, cameraPitch=-30, cameraTargetPosition=[0.1, 0.0, 0.25])


#horz
joint=0
p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 0.0,force=1000,maxVelocity=1)
joint=3
p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 0.0,force=1000,maxVelocity=1)
joint=6
p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 0.0,force=1000,maxVelocity=1)
joint=9
p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 0.0,force=1000,maxVelocity=1)

waitsec(2)



#params
tiltamt=0.02 #0.013 #0.025
fwdstep=0.095
maxVelTilt=0.08 #0.05 #0.02
maxVelLeg=0.4
stepht=0.25
wheelang=np.array([0.0,0.0,0.0,0.0])
maxVel=np.array([1,1,1,1])*6  #2 #4
wtleg=1

def rollwheel(wheelang,maxVel):
    #wheels pos
    #forw
    joint=2
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = wheelang[0],force=1000,maxVelocity=maxVel[0])
    joint=5
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = wheelang[1],force=1000,maxVelocity=maxVel[1])
    joint=8
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = wheelang[2],force=1000,maxVelocity=maxVel[2])
    joint=11
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = wheelang[3],force=1000,maxVelocity=maxVel[3])
    waitsec(1)

def steerwheeldeg(deg):
    rad=np.pi/180*deg
    joint=1
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = -rad,force=1000,maxVelocity=1)
    joint=4
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = rad,force=1000,maxVelocity=1)
    joint=7
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = rad,force=1000,maxVelocity=1)
    joint=10
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = -rad,force=1000,maxVelocity=1)
    waitsec(0.8)

def tiltplatform(tgtpos):
    joint=0
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = tgtpos[0],force=1000,maxVelocity=maxVelTilt)
    joint=3
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = tgtpos[1],force=1000,maxVelocity=maxVelTilt)
    joint=6
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = tgtpos[2],force=1000,maxVelocity=maxVelTilt)
    joint=9
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = tgtpos[3],force=1000,maxVelocity=maxVelTilt)
    waitsec(1) #1

#
#
#
#
#
#



wheelfrw=0.080/0.03
wheelang+=np.array([-wheelfrw,-wheelfrw,-wheelfrw,-wheelfrw])
rollwheel(wheelang,maxVel)




while(1):
    
    #steer 45
    steerwheeldeg(45)
    #wheels pos
    #30deg
    wheel30deg=0.1414*np.pi/6/0.03
    wheelang+=np.array([-wheel30deg,wheel30deg,-wheel30deg,wheel30deg])
    rollwheel(wheelang,maxVel)
    #steer straight
    steerwheeldeg(0)
    
    
    
    
    
    #tilt
    tiltplatform(np.array([-tiltamt, 0.0, 0.0, tiltamt]))
    joint=0
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = stepht,force=1000,maxVelocity=maxVelLeg)
    waitsec(wtleg)
    #wheels pos
    #forw
    wheelfrw=fwdstep/0.03
    wheelang+=np.array([-wheelfrw,-wheelfrw,-wheelfrw,-wheelfrw])
    rollwheel(wheelang,maxVel)
    #horz
    tiltplatform(np.array([stepht, 0.0, 0.0, 0.0]))
    
    
    
    
    
    
    #tilt
    tiltplatform(np.array([stepht, -tiltamt, tiltamt, 0.0]))
    joint=3
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = stepht,force=1000,maxVelocity=maxVelLeg)
    waitsec(wtleg)
    #wheels pos
    #forw
    wheelfrw=fwdstep/0.03
    wheelang+=np.array([-wheelfrw,-wheelfrw,-wheelfrw,-wheelfrw])
    rollwheel(wheelang,maxVel)
    #horz
    tiltplatform(np.array([stepht, stepht, 0.0, 0.0]))
    
    
    
    
    
    
    
    #tilt
    tiltplatform(np.array([stepht, stepht+tiltamt, -tiltamt, 0.0]))
    joint=6
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = stepht,force=1000,maxVelocity=maxVelLeg)
    waitsec(wtleg)
    #wheels pos
    #forw
    wheelfrw=fwdstep/0.03
    wheelang+=np.array([-wheelfrw,-wheelfrw,-wheelfrw,-wheelfrw])
    rollwheel(wheelang,maxVel)
    #horz
    tiltplatform(np.array([stepht, stepht, stepht, 0.0]))
    
    
    
    
    
    
    #tilt
    tiltplatform(np.array([stepht+tiltamt, stepht, stepht, -tiltamt]))
    joint=9
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = stepht,force=1000,maxVelocity=maxVelLeg)
    waitsec(wtleg)
    #wheels pos
    #forw
    wheelfrw=fwdstep/0.03
    wheelang+=np.array([-wheelfrw,-wheelfrw,-wheelfrw,-wheelfrw])
    rollwheel(wheelang,maxVel)
    #horz
    tiltplatform(np.array([stepht, stepht, stepht, stepht]))
    
    
    
    
    
    
    
    
    #steer 45
    steerwheeldeg(45)
    #wheels pos
    #-30deg
    wheel30deg=-0.1414*np.pi/6/0.03
    wheelang+=np.array([-wheel30deg,wheel30deg,-wheel30deg,wheel30deg])
    rollwheel(wheelang,maxVel)
    #steer straight
    steerwheeldeg(0)
    
    
    
    #horz up
    joint=0
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 0.0,force=1000,maxVelocity=maxVelLeg)
    joint=3
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 0.0,force=1000,maxVelocity=maxVelLeg)
    joint=6
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 0.0,force=1000,maxVelocity=maxVelLeg*0.98) #1 0.85, 2 0.92, 6 0.98  
    joint=9
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 0.0,force=1000,maxVelocity=maxVelLeg*0.98)
    waitsec(2)



#wheels pos
#forw
wheelfrw=fwdstep/0.03
wheelang+=np.array([-wheelfrw,-wheelfrw,-wheelfrw,-wheelfrw])
rollwheel(wheelang,maxVel)
#wheels pos
#forw
wheelfrw=fwdstep/0.03
wheelang+=np.array([-wheelfrw,-wheelfrw,-wheelfrw,-wheelfrw])
rollwheel(wheelang,maxVel)



p.disconnect()














##wheels
#joint=2
#p.setJointMotorControl2(platform,joint,p.VELOCITY_CONTROL,targetVelocity = -1.0,force=1000,maxVelocity=1)
#joint=5
#p.setJointMotorControl2(platform,joint,p.VELOCITY_CONTROL,targetVelocity = -1.0,force=1000,maxVelocity=1)
#joint=8
#p.setJointMotorControl2(platform,joint,p.VELOCITY_CONTROL,targetVelocity = -1.0,force=1000,maxVelocity=1)
#joint=11
#p.setJointMotorControl2(platform,joint,p.VELOCITY_CONTROL,targetVelocity = -1.0,force=1000,maxVelocity=1)



#
#joint=1
#p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 1,force=1000,maxVelocity=1)
#joint=4
#p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 1,force=1000,maxVelocity=1)
#joint=7
#p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 1,force=1000,maxVelocity=1)
#joint=10
#p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition = 1,force=1000,maxVelocity=1)


























if (0):
    t0=time.time()
    t=time.time()
    while ((t-t0)<10):
        t=time.time()
    p.disconnect()
    brake    

#Scenery e.g. an inclined box
boxHalfLength = 2.5
boxHalfWidth = 2.5
boxHalfHeight = 0.2
sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
#mass = 1
block=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [-2,0,-0.1],baseOrientation=[0.0,0.1,0.0,1])
sth=0.15
block2=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [1,4.5,-0.2+1*sth],baseOrientation=[0.0,0.0,0.0,1])
block3=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [1,5,-0.2+2*sth],baseOrientation=[0.0,0.0,0.0,1])
block4=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [1,5.5,-0.2+3*sth],baseOrientation=[0.0,0.0,0.0,1])
block5=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [1,6,-0.2+4*sth],baseOrientation=[0.0,0.0,0.0,1])



#Add extra lateral friction to the feet spheres
p.changeDynamics(platform,3,lateralFriction=2)
p.changeDynamics(platform,7,lateralFriction=2)
p.changeDynamics(platform,11,lateralFriction=2)
p.changeDynamics(platform,15,lateralFriction=2)


#Function to calculate roll, hip and knee angles from the x,y,z coords of the foot wrt the hip.
def xyztoang(x,y,z,yoffh,hu,hl):
    dyz=np.sqrt(y**2+z**2)
    lyz=np.sqrt(dyz**2-yoffh**2)
    gamma_yz=-np.arctan(y/z)
    gamma_h_offset=-np.arctan(-yoffh/lyz)
    gamma=gamma_yz-gamma_h_offset
    
    lxzp=np.sqrt(lyz**2+x**2)
    n=(lxzp**2-hl**2-hu**2)/(2*hu)
    beta=-np.arccos(n/hl)
    
    alfa_xzp=-np.arctan(x/lyz)
    alfa_off=np.arccos((hu+n)/lxzp)
    alfa=alfa_xzp+alfa_off
    if any( np.isnan([gamma,alfa,beta])):
        print(x,y,z,yoffh,hu,hl)
    return [gamma,alfa,beta]


def setlegsxyz(xvec,yvec,zvec,vvec):
    #[a1,a2]=xztoang(xvec[0],zvec[0],1,1)
    a=xyztoang(xvec[0]-xhipf,yvec[0]-yhipl,zvec[0],yoffh,hu,hl)  #(x,y,z,yoffh,hu,hl)
    spd=100
    #any(np.isnan(a))
    joint=0
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[0],force=10000,maxVelocity=spd)
    joint=1
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[1],force=10000,maxVelocity=vvec[0])
    joint=2
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[2],force=10000,maxVelocity=vvec[0])

    a=xyztoang(xvec[1]-xhipf,yvec[1]+yhipl,zvec[1],-yoffh,hu,hl)  #(x,y,z,yoffh,hu,hl)
    spd=1.0
    joint=4
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[0],force=10000,maxVelocity=spd)
    joint=5
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[1],force=10000,maxVelocity=vvec[1])
    joint=6
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[2],force=10000,maxVelocity=vvec[1])

    a=xyztoang(xvec[2]-xhipb,yvec[2]-yhipl,zvec[2],yoffh,hu,hl)  #(x,y,z,yoffh,hu,hl)
    spd=1.0
    joint=8
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[0],force=10000,maxVelocity=spd)
    joint=9
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[1],force=10000,maxVelocity=vvec[2])
    joint=10
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[2],force=10000,maxVelocity=vvec[2])

    a=xyztoang(xvec[3]-xhipb,yvec[3]+yhipl,zvec[3],-yoffh,hu,hl)  #(x,y,z,yoffh,hu,hl)
    spd=1.0
    joint=12
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[0],force=10000,maxVelocity=spd)
    joint=13
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[1],force=10000,maxVelocity=vvec[3])
    joint=14
    p.setJointMotorControl2(platform,joint,p.POSITION_CONTROL,targetPosition=a[2],force=10000,maxVelocity=vvec[3])

#Pre-init robot position
setlegsxyz([xhipf,xhipf,xhipb,xhipb],[yhipl+0.1,-yhipl-0.1,yhipl+0.1,-yhipl-0.1],[-0.5,-0.5,-0.5,-0.5],[1,1,1,1])
t0=time.time()
t=time.time()
while ((t-t0)<2):
    t=time.time()


#Rotation matrix for yaw only between robot-frame and world-frame
def RotYawr(yawr):
    Rhor=np.array([[np.cos(yawr),-np.sin(yawr),0], [np.sin(yawr),np.cos(yawr),0], [0,0,1]])
    return Rhor

#Init robot position, orientation and pose params
# O means in world-centered coordinates
# R means in robot-centered coordinates
# r is for "of the robot"
# i is initial
yawri=-1.3
xrOi=np.array([1,6,0.5])
yawri=1.57
xrOi=np.array([2.5,1.04,0.5])
legsRi=np.array([[xhipf,xhipf,xhipb,xhipb],
               [yhipl+0.1,-yhipl-0.1,yhipl+0.1,-yhipl-0.1],
               [-0.5,-0.5,-0.5,-0.5]])
#Set body to the robot pos
xbOi=xrOi
#Init body position and orientation
quat=p.getQuaternionFromEuler([0,0,yawri])
p.resetBasePositionAndOrientation(platform,xbOi,quat)
#Init leg abs pos
Ryawri=RotYawr(yawri)
legsO=(np.dot(Ryawri,legsRi).T + xbOi).T   #Apply rotation plus translation

#Set the non-initial variables and matrix
yawr=yawri
xrO=xrOi
xbO=xrO
Ryawr=RotYawr(yawri)

#Recalc leg rel pos in robot frame and set the legs
dlegsO=(legsO.T-xbO).T
dlegsR=np.dot(Ryawr.T,dlegsO)
setlegsxyz(dlegsR[0],dlegsR[1],dlegsR[2],[1,1,1,1])

#Try small jump
t0=time.time()
t=time.time()
while ((t-t0)<2):
    t=time.time()

#Try small jump
dum=dlegsR
dum[2]=np.array([-0.65, -0.65, -0.65, -0.65])
setlegsxyz(dum[0],dum[1],dum[2],[1000,1000,1000,1000])
t0=time.time()
t=time.time()
while ((t-t0)<2):
    t=time.time()


#Calculate a new robot center position from the average of the feet positions
#Calculate a new robot yaw ditrection also from the feet positions
xfO=(legsO[:,0]+legsO[:,1])/2.0
xbO=(legsO[:,2]+legsO[:,3])/2.0
xrOn=(xfO+xbO)/2.0 + np.array([0,0,0.5])
xfmbO=xfO-xbO
yawrn=np.arctan2(xfmbO[1],xfmbO[0])

#Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
cyaw=10
cpitch=-15
cdist=1.5

#Change general motor speed
vvec=[12]*4

#Current leg to change position (increases to 0 at start)
l=-1
#Init the center for the robot rotation to the current robot pos
xrcO=xrO
#Set the body position to the robot position
xoff=0
yoff=0
#Init to walking fwd
dr=0
drp=0
#Leg sequence (for rotating the robot, I chose to chg legs in the order front-left, fr, br, bl)
lseq=[0,1,3,2]
lseqp=[0,1,3,2]
#lseq=[2,0,3,1]
#lseqp=[2,0,3,1]
t0=time.time()
while (1):
    #Next leg
    l=(l+1)%4
    

    #Apply new walking cycle type (e.g. chg from fwd to bkw) only at the start of next cycle
    if l==0 and (not dr==drp):
        dr=drp
        lseq=lseqp


    ts=time.time()
    t=0
    tf=np.array([0, 1, 2, 3, 4, 5])*0.5
    dtf=tf[1:]-tf[0:-1]
    #One step
    #init step
    xoff=0
    yoff=0
    while t<tf[5]:
        tp=t
        t=time.time()-ts
        delt=t-tp
        #Camera
        cubePos, cubeOrn = p.getBasePositionAndOrientation(platform)
        p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)

        keys = p.getKeyboardEvents()
        #Keys to change camera
        if keys.get(100):  #D
            cyaw+=1
        if keys.get(97):   #A
            cyaw-=1
        if keys.get(99):   #C
            cpitch+=1
        if keys.get(102):  #F
            cpitch-=1
        if keys.get(122):  #Z
            cdist+=.01
        if keys.get(120):  #X
            cdist-=.01

        #Keys to change the robot walk (fwd, bkw, rot right, rot left)
        if keys.get(65297): #Up
            drp=0
        if keys.get(65298): #Down
            drp=2
        if keys.get(65296): #Right
            drp=1
            xrcO=xrO        #Set the center for the robot rotation to the current robot pos
            lseqp=[1,0,2,3] #Change the leg sequence to open up the front arms rather than close them
        if keys.get(65295): #Left
            drp=3
            xrcO=xrO
            lseqp=[0,1,3,2] #Change the leg sequence to open up the front arms rather than close them
    
        
        #Actual leg to move
        k=lseq[l]
           
        #In the beginning of the leg cycle the body is centered at the robot center
        #then it gradually moves in the opposite direction of the leg to be moved 
        #to ensure the center of gravity remains on the other 3 legs
        #when the moving leg goes down again the body center returns to the robot center
        #The vars xoff and yoff move the body w.r.t. the robot center in the robot frame
        if t<tf[1]:
            xoff+=0.003*(-1+2*int(k/2))*delt*70  #Work it out on paper to see it moves opposite to the stepping leg
            yoff+=0.003*(-1+2*(k%2))*delt*50  
    
        elif t>tf[4]:
            xoff-=0.003*(-1+2*int(k/2))*delt*70
            yoff-=0.003*(-1+2*(k%2))*delt*50     
    
        #Recalc leg rel pos in desired robot frame
        dlegsO=(legsO.T-xrO).T  #Translate
        dlegsR=np.dot(Ryawr.T,dlegsO)  #Rotate (Note the inverse rotation is the transposed matrix)
        #Then apply the body movement and set the legs
        setlegsxyz(dlegsR[0]-xoff-0.06,dlegsR[1]-yoff,dlegsR[2],vvec)  #0.03 is for tweaking the center of grav.
        
    
        if t>tf[1] and t<tf[4]:
            dlegsO=(legsO.T-xrcO).T
            yawlO=np.arctan2(dlegsO[1,k],dlegsO[0,k])
            rlO=np.sqrt(dlegsO[0,k]**2+dlegsO[1,k]**2)
            
            if dr==0:
                if t>tf[2] and t<tf[3]:
                    legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]+0.015*np.cos(yawr)*delt*25
                    legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]+0.015*np.sin(yawr)*delt*25
                    xrcO=xrO        #Set the center for the robot rotation to the current robot pos
            elif dr==1:
                yawlO-=0.015*delt*25 
                legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]
                legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]
            elif dr==2:
                legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]-0.01*np.cos(yawr)*delt*25
                legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]-0.01*np.sin(yawr)*delt*25
                xrcO=xrO        #Set the center for the robot rotation to the current robot pos
            elif dr==3:
                yawlO+=0.015*delt*25 
                legsO[0,k]=rlO*np.cos(yawlO)+xrcO[0]
                legsO[1,k]=rlO*np.sin(yawlO)+xrcO[1]
            
            if t>tf[1] and t<tf[2]:
                #Move leg k upwards 
                legsO[2,k]+=.025*delt*25
            elif t>tf[3] and t<tf[4]:
                #Move leg k wards 
                #if legsO[2,k]>-0.0:
                #    legsO[2,k]-=.060*delt*25
                dum=p.getLinkState(platform,4*k+3)
                xft=dum[0]
                dum=p.rayTest(xft-np.array([0,0,0.041]),xft-np.array([0,0,0.50]))
                
                dum2,dum3=p.getBasePositionAndOrientation(platform)
                dum4=p.getEulerFromQuaternion(dum3)
                #print(p.getEulerFromQuaternion(dum3))
                
                
                if (dum[0][2]>0.09 and dum[0][2]<0.99 and legsO[2,k]>0):
                    legsO[2,k]-=.04*delt*25
                elif (k==0 or k==2) and dum4[1]<-0.03 and legsO[2,k]>0:
                    legsO[2,k]-=.01*delt*25
                elif (k==1 or k==3) and dum4[1]>0.03 and legsO[2,k]>0: 
                    legsO[2,k]-=.01*delt*25
                    
            if k==0:
                dum=p.getLinkState(platform,4*k+3)
                xft=dum[0]
                #dum=p.getOverlappingObjects(xft-np.array([0.01,0.01,0.08]),xft-np.array([0.01,0.01,0.07]))
                #if len(dum)>2:
                #    print(dum)
                dum=p.rayTest(xft-np.array([0,0,0.041]),xft-np.array([0,0,0.50]))
                #print(tv%200,dum[0][2],legsO[2,k])
                
    #            dum=p.getLinkState(platform,4*k+3)
    #            xft=dum[0]
    #            dum=p.rayTest(xft-np.array([0,0,0.05]),xft-np.array([0,0,0.55]))
    #            #print(dum[0][2])
    #            if dum[0][2]>0.1 and dum[0][2]<0.9:
    #                if legsO[2,k]>0.016:
    #                    legsO[2,k]-=.016
    #                else:
    #                    dum=[0,1,2,3]
    #                    dum.remove(k)
    #                    for m in dum:
    #                        legsO[2,m]+=.032
                            
                #p.rayTest()
        else:
            #Move/keep all legs down to the ground
            for m in [0,1,2,3]:
                a=1
                #legsO[2,m]=0.0
                #if legsO[2,m] > 0.05:
                #    legsO[2,m]=0.05
            
        
        #Calculate vectors and matrix for the next loop
        xfrO=(legsO[:,0]+legsO[:,1])/2.0
        xbkO=(legsO[:,2]+legsO[:,3])/2.0
        xrO=(xfrO+xbkO)/2.0 
        xrO[2]=0.5
        xfmbO=xfrO-xbkO
        yawr=np.arctan2(xfmbO[1],xfmbO[0])
        Ryawr=RotYawr(yawr)
    
        #dum2,dum3=p.getBasePositionAndOrientation(platform)
        
        sm=0
        for n in range(4):
            dum=p.getLinkState(platform,4*n+3)
            sm+=np.array(dum[0])/4.0
        
            
        #print(sm[0]-xrO[0],sm[1]-xrO[1])
        print(delt)
        time.sleep(0.02)
	
p.disconnect()
    






















