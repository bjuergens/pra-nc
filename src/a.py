 #!/usr/bin/env python3

import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)

p.setGravity(0, 0, -10)
planeId = p.loadSDF("src/my_room.sdf")
# planeId = p.loadSDF("src/virtual_room.sdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadSDF("src/model.sdf")
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId[0])

useRealTimeSimulation = 0

if (useRealTimeSimulation):
  p.setRealTimeSimulation(1)

while 1:
  if (useRealTimeSimulation):
    p.setGravity(0, 0, -10)
    sleep(0.01)  # Time in seconds.
  else:
    p.stepSimulation()
