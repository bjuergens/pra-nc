 #!/usr/bin/env python3

import pybullet as p
import time
import pybullet_data


physicsClient = p.connect(p.GUI) # p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

husky = p.loadURDF("husky/husky.urdf",cubeStartPos, cubeStartOrientation)

cubePos, cubeOrn = p.getBasePositionAndOrientation(husky)

planeId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1,0.25,1])
planeVisId = p.createVisualShape(p.GEOM_BOX, halfExtents=[1,0.25,1],  rgbaColor=[1, 1, 0, 1])

planeA = p.createMultiBody(baseCollisionShapeIndex=planeId, basePosition=[0,5,2], baseVisualShapeIndex=planeVisId)
planeB = p.createMultiBody(baseCollisionShapeIndex=planeId, basePosition=[0,-5,2], baseVisualShapeIndex=planeVisId)

green=[0, 1, 0, 1]
blue=[0, 0, 1, 1]
p.changeVisualShape(planeA, -1, rgbaColor=[0, 1, 0, 1])
p.changeVisualShape(planeB, -1, rgbaColor=[0, 0, 1, 1])

p.setRealTimeSimulation(0)


targetVelocitySlider = p.addUserDebugParameter("wheelVelocity",-15,15,0)
maxForceSlider = p.addUserDebugParameter("maxForce",0,150,100)

camInfo = p.getDebugVisualizerCamera()

class my_husky(object):

    def __init__(self):
        self.lastCamTime = time.time()
        self.lastConTime = time.time()
        for joint in range(p.getNumJoints(husky)):
            if 1:
                print("joint[",joint,"]=", p.getJointInfo(husky,joint))
            if p.getJointInfo(husky,joint)[1]==b'user_rail':
                self.zed_camera_joint = joint
            if p.getJointInfo(husky,joint)[1]==b'front_left_wheel':
                self.front_left_wheel = joint
            if p.getJointInfo(husky,joint)[1]==b'front_right_wheel':
                self.front_right_wheel = joint
            if p.getJointInfo(husky,joint)[1]==b'rear_left_wheel':
                self.rear_left_wheel = joint
            if p.getJointInfo(husky,joint)[1]==b'rear_right_wheel':
                self.rear_right_wheel = joint

    def update_control(self):
        now = time.time()

        if now - self.lastConTime > .1:
            maxForce = p.readUserDebugParameter(maxForceSlider)
            targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
            p.setJointMotorControl2(husky,self.front_left_wheel,p.VELOCITY_CONTROL,targetVelocity=-targetVelocity,force=maxForce)
            p.setJointMotorControl2(husky,self.rear_left_wheel,p.VELOCITY_CONTROL,targetVelocity=-targetVelocity,force=maxForce)
            p.setJointMotorControl2(husky,self.front_right_wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
            p.setJointMotorControl2(husky,self.rear_right_wheel,p.VELOCITY_CONTROL,targetVelocity=targetVelocity,force=maxForce)
            self.lastConTime = now

    def update_cam(self):
        # render Camera at 10Hertz
        now = time.time()
        if now - self.lastCamTime > .1:
            ls = p.getLinkState(husky, self.zed_camera_joint, computeForwardKinematics=True)
            camPos = ls[0]
            camOrn = ls[1]
            camMat = p.getMatrixFromQuaternion(camOrn)
            forwardVec = [camMat[0], camMat[3], camMat[6]]
            camUpVec = [camMat[2], camMat[5], camMat[8]]
            camTarget = [camPos[0] + forwardVec[0] * 10, camPos[1] + forwardVec[1] * 10, camPos[2] + forwardVec[2] * 10]
            viewMat = p.computeViewMatrix(camPos, camTarget, camUpVec)
            projMat = camInfo[3]
            p.getCameraImage(320, 200, viewMatrix=viewMat, projectionMatrix=projMat, renderer=p.ER_BULLET_HARDWARE_OPENGL)
            self.lastCamTime = now


my_robot = my_husky()
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

    my_robot.update_cam()
    my_robot.update_control()
    p.stepSimulation()
    time.sleep(0.0005)

