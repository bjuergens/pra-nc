#!/usr/bin/env python3
import numpy as np
import pybullet as p
import pybullet_data

class Husky(object):
    def __init__(self):
        self.husky_model = p.loadURDF("husky/husky.urdf",
                                      basePosition=[0, 0, 0.1],
                                      baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

        for joint in range(p.getNumJoints(self.husky_model)):
            if p.getJointInfo(self.husky_model, joint)[1] == b'user_rail':
                self.zed_camera_joint = joint

    def update_cam(self):
        joint_pos, joint_orn, _, _, _, _ = p.getLinkState(self.husky_model, self.zed_camera_joint,
                                                          computeForwardKinematics=True)
        cam_rot = np.array(p.getMatrixFromQuaternion(joint_orn)).reshape(3, 3)
        cam_direction = cam_rot.dot((1, 0, 0))
        cam_up_vec = cam_rot.dot((0, 0, 1))
        cam_pos = joint_pos
        # no freeze, when camera pos is slighty behind joint
        # cam_pos = joint_pos - (cam_direction * 1.0)
        cam_target = joint_pos + (cam_direction * 1.0)
        view_mat = p.computeViewMatrix(cam_pos, cam_target, cam_up_vec)
        proj_mat = p.computeProjectionMatrixFOV(100, 80 / 80, 0.1, 100)
        print("getCameraImage...")
        img = p.getCameraImage(80, 80, viewMatrix=view_mat, projectionMatrix=proj_mat, renderer=p.ER_TINY_RENDERER)
        print("getCameraImage... done")
        return img


physicsClient = p.connect(p.DIRECT)  # p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -15)
p.loadURDF("plane.urdf")

my_robot = Husky()

step = 0
while 1:
    step = step + 1
    print(step)
    if step % 10 == 0:
        my_robot.update_cam()
    p.stepSimulation()

