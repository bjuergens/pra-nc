#!/usr/bin/env python3

import time

import numpy as np
import pybullet as p
import pybullet_data



class World(object):
    green = [0, 1, 0, 1]
    blue = [0, 0, 1, 1]
    red = [1, 0, 0, 1]

    count_steps = 0
    count_switch = 0

    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        p.setGravity(0, 0, -10)
        p.loadURDF("plane.urdf")

        plane_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 0.25, 1])
        plane_vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[1, 0.25, 1], rgbaColor=[1, 1, 0, 1])

        self.planeA = p.createMultiBody(baseCollisionShapeIndex=plane_id, basePosition=[0, 5, 2],
                                        baseVisualShapeIndex=plane_vis_id)
        self.planeB = p.createMultiBody(baseCollisionShapeIndex=plane_id, basePosition=[0, -5, 2],
                                        baseVisualShapeIndex=plane_vis_id)

        p.changeVisualShape(self.planeA, -1, rgbaColor=self.blue)
        p.changeVisualShape(self.planeB, -1, rgbaColor=self.blue)

        p.setRealTimeSimulation(0)

    def do_step(self):
        self.count_steps = self.count_steps + 1
        if self.count_steps % 5000 == 1:
            self.count_switch = self.count_switch + 1
            if self.count_switch % 2 == 0:
                p.changeVisualShape(self.planeA, -1, rgbaColor=self.green)
                p.changeVisualShape(self.planeB, -1, rgbaColor=self.red)
            else:
                p.changeVisualShape(self.planeA, -1, rgbaColor=self.red)
                p.changeVisualShape(self.planeB, -1, rgbaColor=self.green)

        p.stepSimulation()


class Brain(object):
    def __init__(self):
        self.min_red_ratio = 1000

    def process(self, count_red_left, count_red_right, count_non_red):
        if count_red_left + count_red_right < count_non_red / self.min_red_ratio:
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


class Husky(object):
    maxForce = 0
    targetVelocity = 0

    class Camera(object):
        init_camera_vector = (1, 0, 0)
        init_up_vector = (0, 0, 1)

        def __init__(self, pixel_width=80, pixel_height=80, near_plane=0.1, far_plane=100, fov=100):
            self.pixel_width = pixel_width
            self.pixel_height = pixel_height
            self.projection_mat = p.computeProjectionMatrixFOV(fov, pixel_width / pixel_height, near_plane, far_plane)

    def __init__(self, brain):
        self.brain = brain
        self.camera = Husky.Camera()
        self.husky_model = p.loadURDF("husky/husky.urdf",
                                      basePosition=[0, 0, 1],
                                      baseOrientation=p.getQuaternionFromEuler([0, 0, 0]))

        for joint in range(p.getNumJoints(self.husky_model)):
            # print("joint[", joint, "]=", p.getJointInfo(self.husky_model, joint))
            if p.getJointInfo(self.husky_model, joint)[1] == b'user_rail':
                self.zed_camera_joint = joint
            if p.getJointInfo(self.husky_model, joint)[1] == b'front_left_wheel':
                self.front_left_wheel = joint
            if p.getJointInfo(self.husky_model, joint)[1] == b'front_right_wheel':
                self.front_right_wheel = joint
            if p.getJointInfo(self.husky_model, joint)[1] == b'rear_left_wheel':
                self.rear_left_wheel = joint
            if p.getJointInfo(self.husky_model, joint)[1] == b'rear_right_wheel':
                self.rear_right_wheel = joint

    def update(self):
        img = self.update_cam()
        count_result = self.count_red_pixels(img)
        command_left, command_right = self.brain.process(*count_result)
        self.update_control(command_left, command_right)
        return img

    def update_control(self, command_left, command_right):
        p.setJointMotorControl2(self.husky_model, self.front_left_wheel, p.VELOCITY_CONTROL,
                                targetVelocity=command_left * self.targetVelocity,
                                force=self.maxForce)
        p.setJointMotorControl2(self.husky_model, self.rear_left_wheel, p.VELOCITY_CONTROL,
                                targetVelocity=command_left * self.targetVelocity,
                                force=self.maxForce)
        p.setJointMotorControl2(self.husky_model, self.front_right_wheel, p.VELOCITY_CONTROL,
                                targetVelocity=command_right * self.targetVelocity,
                                force=self.maxForce)
        p.setJointMotorControl2(self.husky_model, self.rear_right_wheel, p.VELOCITY_CONTROL,
                                targetVelocity=command_right * self.targetVelocity,
                                force=self.maxForce)

    def update_cam(self):
        joint_pos, joint_orn, _, _, _, _ = p.getLinkState(self.husky_model, self.zed_camera_joint,
                                                          computeForwardKinematics=True)
        cam_rot = np.array(p.getMatrixFromQuaternion(joint_orn)).reshape(3, 3)
        cam_direction = cam_rot.dot(self.camera.init_camera_vector)
        cam_up_vec = cam_rot.dot(self.camera.init_up_vector)
        # note: if cam is exactly on joint, getCameraImage freezes sometimes
        cam_pos = joint_pos - (cam_direction * 1.0)
        cam_target = joint_pos + (cam_direction * 1.0)
        view_mat = p.computeViewMatrix(cam_pos, cam_target, cam_up_vec)

        return p.getCameraImage(self.camera.pixel_width,
                                self.camera.pixel_height,
                                viewMatrix=view_mat,
                                projectionMatrix=self.camera.projection_mat,
                                flags=p.ER_NO_SEGMENTATION_MASK)

    @staticmethod
    def count_red_pixels(img):
        """detect left red and right red like nrp
        pixelformat is guessed, but it seems to work"""
        w = img[0]  # width of the image, in pixels
        h = img[1]  # height of the image, in pixels
        rgb_buffer = img[2]  # color data RGB
        # dep_buffer = img_arr[3]  # depth data
        count_red_right = 0
        count_red_left = 0
        count_non_red = 0
        for y in range(h):
            for x in range(w):
                # numpy image
                r, g, b, a = (rgb_buffer[y][x][0], rgb_buffer[y][x][1], rgb_buffer[y][x][2], rgb_buffer[y][x][3])

                if r > g and r > b:
                    if x > w / 2:
                        count_red_right = count_red_right + 1
                    else:
                        count_red_left = count_red_left + 1
                else:
                    count_non_red = count_non_red + 1
        return count_red_left, count_red_right, count_non_red

print("import done")

if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)  # p.DIRECT for non-graphical version
    my_word = World()
    my_robot = Husky(brain=Brain())
    while 1:
        my_word.do_step()
        if my_word.count_steps % 20 == 0:
            print(my_word.count_steps)
            my_robot.maxForce = 150
            my_robot.targetVelocity = 20
            my_robot.update()

        time.sleep(0.0005)
