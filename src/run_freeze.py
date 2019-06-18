#!/usr/bin/env python3
import pybullet as p
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)  # p.DIRECT for non-graphical version

class World(object):
    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        p.setGravity(0, 0, -15)
        p.loadURDF("plane.urdf")

    def loop(self, robot):
        step = 0
        while 1:
            step = step + 1
            print("update_cam")
            if step % 10 == 0:
                robot.update_cam()
            print("stepSimulation")
            p.stepSimulation()
            print(step)


class Husky(object):

    def __init__(self):
        self.husky_model = p.loadURDF("husky/husky.urdf",
                                      basePosition=[0, 0, 1],
                                      baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

        for joint in range(p.getNumJoints(self.husky_model)):
            if p.getJointInfo(self.husky_model, joint)[1] == b'user_rail':
                self.zed_camera_joint = joint

    def update_cam(self):
        ls = p.getLinkState(self.husky_model, self.zed_camera_joint, computeForwardKinematics=True)
        cam_pos = [ls[0][0],ls[0][1],ls[0][2]]
        cam_orn = ls[1]
        cam_mat = p.getMatrixFromQuaternion(cam_orn)
        # forward_vec = [cam_mat[0], cam_mat[3], cam_mat[6]]
        forward_vec = cam_orn
        cam_up_vec = [cam_mat[2], cam_mat[5], cam_mat[8]]
        cam_target = [cam_pos[0] + forward_vec[0] * 10,
                      cam_pos[1] + forward_vec[1] * 10,
                      cam_pos[2] + forward_vec[2] * 10]

        view_mat = p.computeViewMatrix(cam_pos, cam_target, cam_up_vec)

        com_p, com_o, _, _, _, _ = p.getLinkState(self.husky_model, self.zed_camera_joint, computeForwardKinematics=True)
        rot_matrix = p.getMatrixFromQuaternion(com_o)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        init_camera_vector = (0, 0, 1)  # z-axis
        init_up_vector = (0, 1, 0)  # y-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)

        pixelWidth = 80
        pixelHeight = 80
        # this proj_mat is what the debug-renderer uses. It seems to work
        proj_mat = (
        0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0,
        -0.02000020071864128, 0.0)


        print("getCameraImage2")
        return p.getCameraImage(pixelWidth, pixelHeight, viewMatrix=view_mat, projectionMatrix=proj_mat,
flags=p.ER_NO_SEGMENTATION_MASK)


my_word = World()
my_robot = Husky()
my_word.loop(my_robot)
