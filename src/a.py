 #!/usr/bin/env python3

import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI) # p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

boxId = p.loadURDF("husky/husky.urdf",cubeStartPos, cubeStartOrientation)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

planeId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1,0.25,1])
planeVisId = p.createVisualShape(p.GEOM_BOX, halfExtents=[1,0.25,1],  rgbaColor=[1, 1, 0, 1])

planeA = p.createMultiBody(baseCollisionShapeIndex=planeId, basePosition=[0,5,2], baseVisualShapeIndex=planeVisId)
planeB = p.createMultiBody(baseCollisionShapeIndex=planeId, basePosition=[0,-5,2], baseVisualShapeIndex=planeVisId)

green=[0, 1, 0, 1]
blue=[0, 0, 1, 1]
p.changeVisualShape(planeA, -1, rgbaColor=[0, 1, 0, 1])
p.changeVisualShape(planeB, -1, rgbaColor=[0, 0, 1, 1])

p.setRealTimeSimulation(0)

count=0

while 1:
    count = count +1
    if count%10==0:
        if count%2==0:
            p.changeVisualShape(planeA, -1, rgbaColor=blue)
            p.changeVisualShape(planeB, -1, rgbaColor=green)
        else:
            p.changeVisualShape(planeA, -1, rgbaColor=green)
            p.changeVisualShape(planeB, -1, rgbaColor=blue)
    p.stepSimulation()
    sleep(0.005)
