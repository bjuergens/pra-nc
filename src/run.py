#!/usr/bin/env python3

import time

import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)  # p.DIRECT for non-graphical version


class world(object):
    green = [0, 1, 0, 1]
    blue = [0, 0, 1, 1]
    red = [1, 0, 0, 1]

    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        p.setGravity(0, 0, -10)
        p.loadURDF("plane.urdf")

        planeId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.25, 1])
        planeVisId = p.createVisualShape(p.GEOM_BOX, halfExtents=[1, 0.25, 1], rgbaColor=[1, 1, 0, 1])

        self.planeA = p.createMultiBody(baseCollisionShapeIndex=planeId, basePosition=[0, 5, 2],
                                        baseVisualShapeIndex=planeVisId)
        self.planeB = p.createMultiBody(baseCollisionShapeIndex=planeId, basePosition=[0, -5, 2],
                                        baseVisualShapeIndex=planeVisId)

        p.changeVisualShape(self.planeA, -1, rgbaColor=[0, 1, 0, 1])
        p.changeVisualShape(self.planeB, -1, rgbaColor=[0, 0, 1, 1])

        p.setRealTimeSimulation(0)

        self.targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -50, 20, 50)
        self.maxForceSlider = p.addUserDebugParameter("maxForce", 0, 150, 100)

    def loop(self, robot):
        count_step = 0
        last_robot_update_time = time.time()
        count_switch = 0
        last_switch_time = 0
        while 1:
            count_step = count_step + 1
            now = time.time()

            if now - last_switch_time > 5:
                last_switch_time = now
                count_switch = count_switch + 1
                print(count_switch)
                if count_switch % 2 == 0:
                    p.changeVisualShape(self.planeA, -1, rgbaColor=self.green)
                    p.changeVisualShape(self.planeB, -1, rgbaColor=self.red)
                else:
                    p.changeVisualShape(self.planeA, -1, rgbaColor=self.red)
                    p.changeVisualShape(self.planeB, -1, rgbaColor=self.green)

            # render Camera at 10Hertz
            if now - last_robot_update_time > .1:
                robot.maxForce = p.readUserDebugParameter(self.maxForceSlider)
                robot.targetVelocity = p.readUserDebugParameter(self.targetVelocitySlider)
                robot.update()
                last_robot_update_time = now
            p.stepSimulation()
            time.sleep(0.0005)


class my_bain(object):

    def __init__(self):
        pass

    def process(self, count_red_left, count_red_right, count_non_red):
        if count_red_left + count_red_right < count_non_red / 1000:
            left_speed = -0.8
            right_speed = 0.8
        else:
            if count_red_left > count_red_right:
                left_speed = 0.6
                right_speed = 1
            else:
                left_speed = 1.0
                right_speed = 0.6

        return left_speed, right_speed


class husky(object):
    maxForce = 0
    targetVelocity = 0

    def __init__(self, brain):
        self.brain = brain
        cubeStartPos = [0, 0, 1]
        cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.camInfo = p.getDebugVisualizerCamera()

        self.husky = p.loadURDF("husky/husky.urdf", cubeStartPos, cubeStartOrientation)

        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.husky)
        for joint in range(p.getNumJoints(self.husky)):
            if 1:
                print("joint[", joint, "]=", p.getJointInfo(self.husky, joint))
            if p.getJointInfo(self.husky, joint)[1] == b'user_rail':
                self.zed_camera_joint = joint
            if p.getJointInfo(self.husky, joint)[1] == b'front_left_wheel':
                self.front_left_wheel = joint
            if p.getJointInfo(self.husky, joint)[1] == b'front_right_wheel':
                self.front_right_wheel = joint
            if p.getJointInfo(self.husky, joint)[1] == b'rear_left_wheel':
                self.rear_left_wheel = joint
            if p.getJointInfo(self.husky, joint)[1] == b'rear_right_wheel':
                self.rear_right_wheel = joint

    def update(self):
        img = self.update_cam()
        count_result = self.count_red_pixels(img)
        command_left, command_right = self.brain.process(*count_result)
        self.update_control(command_left, command_right)

    def update_control(self, command_left, command_right):
        p.setJointMotorControl2(self.husky, self.front_left_wheel, p.VELOCITY_CONTROL,
                                targetVelocity=command_left * self.targetVelocity,
                                force=self.maxForce)
        p.setJointMotorControl2(self.husky, self.rear_left_wheel, p.VELOCITY_CONTROL,
                                targetVelocity=command_left * self.targetVelocity,
                                force=self.maxForce)
        p.setJointMotorControl2(self.husky, self.front_right_wheel, p.VELOCITY_CONTROL,
                                targetVelocity=command_right * self.targetVelocity,
                                force=self.maxForce)
        p.setJointMotorControl2(self.husky, self.rear_right_wheel, p.VELOCITY_CONTROL,
                                targetVelocity=command_right * self.targetVelocity,
                                force=self.maxForce)

    def update_cam(self):
        ls = p.getLinkState(self.husky, self.zed_camera_joint, computeForwardKinematics=True)
        camPos = ls[0]
        camOrn = ls[1]
        camMat = p.getMatrixFromQuaternion(camOrn)
        forwardVec = [camMat[0], camMat[3], camMat[6]]
        camUpVec = [camMat[2], camMat[5], camMat[8]]
        camTarget = [camPos[0] + forwardVec[0] * 10, camPos[1] + forwardVec[1] * 10, camPos[2] + forwardVec[2] * 10]
        viewMat = p.computeViewMatrix(camPos, camTarget, camUpVec)
        projMat = self.camInfo[3]
        # getCameraImage seems to update the debug-view but I don't know why and how
        return p.getCameraImage(80, 80, viewMatrix=viewMat, projectionMatrix=projMat,
                                renderer=p.ER_BULLET_HARDWARE_OPENGL)

    def count_red_pixels(self, img_arr):
        """detect left red and right red like nrp
        pixelformat is guessed, but it seems to work"""
        w = img_arr[0]  # width of the image, in pixels
        h = img_arr[1]  # height of the image, in pixels
        rgbBuffer = img_arr[2]  # color data RGB
        depBuffer = img_arr[3]  # depth data
        count_red_right = 0
        count_red_left = 0
        count_non_red = 0
        for y in range(h):
            for x in range(w):
                pos = (y * w + x) * 4
                r, g, b, a = (rgbBuffer[pos], rgbBuffer[pos + 1], rgbBuffer[pos + 2], rgbBuffer[pos + 3])
                if r > g and r > b:
                    if x > w / 2:
                        count_red_right = count_red_right + 1
                    else:
                        count_red_left = count_red_left + 1
                else:
                    count_non_red = count_non_red + 1
        return count_red_left, count_red_right, count_non_red


my_word = world()
my_robot = husky(brain=my_bain())
my_word.loop(my_robot)
