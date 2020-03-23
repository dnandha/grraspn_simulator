# fixed camera setup

import pybullet as p
import numpy as np
from PIL import Image
import random
import os
import util as u
from data import H, W

class CameraConfig(object):

    def __init__(self, seed=None):
        # TODO: push config of these up
        self.width = W
        self.height = H

        self.fov = 60

        # focus a bit above center of tray
        #self.camera_target = np.array([0.5, 0.1, 0.1])
        self.camera_target = np.array([0.8, 0.1, 0.1])
        self.camera_target = list(self.camera_target)

        self.distance = 0.2

        # yaw=0 => left hand side, =90 towards arm, =180 from right hand side
        self.yaw = 90

        # pitch=0 looking horizontal, we pick a value looking slightly down
        self.pitch = -60

        self.light_color = [0.8]*3

        self.light_direction = [-1,1,1]


class Camera(object):

    def __init__(self, camera_id, img_dir, joint_info_file, kuka_uid, fixed_config=None):
        self.id = camera_id
        self.img_dir = img_dir
        if joint_info_file == None:
            self.joint_info_file = None
        else:
            self.joint_info_file = open(joint_info_file, "w")
        self.kuka_uid = kuka_uid
        self.fixed_config = fixed_config
        if not os.path.exists(img_dir):
            os.makedirs(img_dir)

    def render(self):#, frame_num):
        # use fixed config (if supplied) otherwise generate
        # a new one for this render
        if self.fixed_config is None:
            config = CameraConfig()
        else:
            config = self.fixed_config

        proj_matrix = p.computeProjectionMatrixFOV(fov=config.fov,
                                                   aspect=float(config.width) / config.height,
                                                   nearVal=0.1,
                                                   farVal=100.0)

        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=config.camera_target,
                                                          distance=config.distance,
                                                          yaw=config.yaw,
                                                          pitch=config.pitch,
                                                          roll=0,  # varying this does nothing (?)
                                                          upAxisIndex=2)

        # call bullet to render
        rendering = p.getCameraImage(width=config.width, height=config.height,
                                     viewMatrix=view_matrix,
                                     projectionMatrix=proj_matrix,
                                     lightColor=config.light_color,
                                     lightDirection=config.light_direction,
                                     shadow=1,
                                     renderer=p.ER_BULLET_HARDWARE_OPENGL)

        ## convert RGB to PIL image
        rgb_array = np.array(rendering[2], dtype=np.uint8)
        rgb_array = rgb_array.reshape((config.height, config.width, 4))
        rgb_array = rgb_array[:, :, :3]
        bgr_array = rgb_array[:, :, ::-1]

        depth_array = np.array(rendering[3], dtype=np.float32)
        depth_array = depth_array.reshape((config.height, config.width, 1))

        return bgr_array, depth_array, np.array(view_matrix).reshape((4,4)).T, np.array(proj_matrix).reshape((4,4)).T, config.width, config.height

        ## save image
        #output_fname = "%s/%s" % (self.img_dir, u.frame_filename_format(frame_num))
        #print("output_fname", output_fname)
        #img.save(output_fname)

        # capture joint states
        #if self.joint_info_file is not None:
        #    joint_info_output = [frame_num]
        #    for j in range(p.getNumJoints(self.kuka_uid)):
        #        # output just position (0th) element, ignore velocity, torques etc
        #        joint_info_output.append(p.getJointState(self.kuka_uid, j)[0])
        #    print("\t".join(map(str, joint_info_output)), file=self.joint_info_file)
