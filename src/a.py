 #!/usr/bin/env python3

import pybullet as p
from time import sleep

physicsClient = p.connect(p.GUI)

p.setGravity(0, 0, -10)
# planeId = p.loadSDF("src/my_room.sdf")
# planeId = p.loadSDF("bullet3/examples/pybullet/gym/pybullet_data/table/table.urdf")

p.loadURDF("bullet3/data/plane.urdf")
#%%
# r2d2=p.loadURDF("r2d2.urdf",[0,0,0.5])

# planeId = p.loadSDF("src/virtual_room.sdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# boxId_orig = p.loadSDF("src/model.sdf")
boxId = p.loadURDF("bullet3/examples/pybullet/gym/pybullet_data/husky/husky.urdf",[0,0,0.5])

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

useRealTimeSimulation = 0

if (useRealTimeSimulation):
  p.setRealTimeSimulation(1)

while 1:
  if (useRealTimeSimulation):
    p.setGravity(0, 0, -10)
    sleep(0.01)  # Time in seconds.
  else:
    p.stepSimulation()
    sleep(0.01)  # Time in seconds.
