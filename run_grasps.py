#!/usr/bin/env python3

import argparse
import camera
import math
import numpy as np
import pybullet as p
import random
import time
import util as u
import kuka_env
import cv2
import os

from detection_webservice.client import Client, draw_preds


parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--img-dir', type=str, default='imgs',
                    help='base dir for output. images are save to {img_dir}/rNNN/cNNN/fNNNN.png')
#parser.add_argument('--joint-info-dir', type=str, default='joint_infos',
#                    help="base dir for joint info. joint info is saved to {joint_info_dir}/rNNN/cNNN.tsv"
#                         " None => don't save joint info")
parser.add_argument('--run', type=int, default=1, help='run_id for img saving')
parser.add_argument('--num-trials', type=int, default=100, help='number of trials to run')
#parser.add_argument('--fixed-camera-configs', action='store_true', help='if set, have fixed camera configs')
#parser.add_argument('--fixed-camera-seed-offset', type=int, default=0,
#                    help='offset to add to seed when generating fixed cameras')
parser.add_argument('--num-objects', type=int, default=1, help='number of objects in tray')
#parser.add_argument('--obj-urdf-dir', type=str, default='./objs', help='base dir for procedural objs')
parser.add_argument('--urdf-dir', type=str, default='./models/ycb', help='base dir for procedural objs')
parser.add_argument('--heuristic', action='store_true', help='use heuristic for prediction selection')
parser.add_argument('--gui', action='store_true', help='if set, run with bullet explorer gui')
opts = parser.parse_args()
print("opts", opts)


######################################################################################

# OFFSETS
# TODO: don't hardcode
LIFT_OFFS = 0.4
MIN_JAW = 30

# create client to prediction server
cloud = Client()

# create robot environment
kuka_env = kuka_env.KukaEnv(opts.gui, opts.urdf_dir)

# setup environment
camera_img_dir = "/".join([opts.img_dir, u.run_dir_format(opts.run), u.camera_dir_format(0)])
camera = camera.Camera(camera_id=0,
                      img_dir=camera_img_dir,
                      joint_info_file=None,
                      kuka_uid=kuka_env.kuka.kukaUid)
                      #fixed_config=fixed_config)


#print(opts.fixed_camera_configs)
#if opts.fixed_camera_configs:
#    fixed_config = camera.RandomCameraConfig(seed=i+opts.fixed_camera_seed_offset)
#else:
#    fixed_config = None

#if opts.joint_info_dir is None:
#    camera_joint_info_file = None
#else:
#    camera_joint_info_file = "/".join([opts.joint_info_dir, u.run_dir_format(opts.run), u.camera_dir_format(0)]) + ".tsv"
#    u.ensure_dir_exists_for_file(camera_joint_info_file)
#
#    camera = camera.Camera(camera_id=0,
#                          img_dir=camera_img_dir,
#                          joint_info_file=camera_joint_info_file,
#                          kuka_uid=kuka_env.kuka.kukaUid,
#                          fixed_config=fixed_config)

# grasp loop
top_1 = [0, 0]
top_2 = [0, 0]
top_3 = [0, 0]
for _ in range(opts.num_trials+1):
    kuka_env.drop_objects(opts.num_objects)
    for _ in range(100):
        p.stepSimulation()
    
    time.sleep(1)

    # take picture
    img, depth, view_mat, proj_mat, width, height = camera.render()

    view_inv = np.linalg.inv(view_mat)
    proj_inv = np.linalg.inv(proj_mat)

    #camera_pos = view_inv[:3,-1]
    #f_x = proj_mat[0,0]
    #f_y = proj_mat[1,1]
    #c_x = proj_mat[2,1]
    #c_y = proj_mat[1,1]

    # RQ decomp to reconstruct camera mat from proj mat
    #R, K = np.linalg.qr(proj_mat[:,:3])
    #T = np.diag(np.sign(np.diag(K)))
    #if np.linalg.det(T) < 0:
    #    T[1,1] *= -1
    #K = np.dot(K, T)
    #K = K / K[-1,-1]

    
    # consult prediction server
    preds = cloud.send_array(img)

    if len(preds) < 1:
        continue

    # heuristic: sort predictions by distance to center 
    if opts.heuristic:
        preds = np.array(preds)
        X, Y = preds[:,0], preds[:,1]
        x_, y_ = np.mean(X), np.mean(Y)
        dists = []
        for pred in preds:
            dist = np.linalg.norm((np.array(pred[0], pred[1]) - np.array(x_, y_)))
            dists.append(dist)
        preds = preds[np.argsort(dists, kind='mergesort')]

    # save prediction
    img = draw_preds(img, preds[:3])
    cv2.imwrite(os.path.join(camera_img_dir, "grasp.png"), img)

    state_id = p.saveState()
    for i, pred in enumerate(preds):
        xx, yy, ww, hh, aa = pred
        if ww < MIN_JAW:
            continue
        p.restoreState(state_id)

        theta = a * np.pi / 180.0
        cos, sin = np.cos(theta), np.sin(theta)
        rect = [(-w / 2, h / 2), (-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2)]
        rot_rect = [(int(sin * yy + cos * xx + x), int(cos * yy - sin * xx + y))
                for (xx, yy) in rect]
        zz = depth[int(yy), int(xx), 0]

        # raster (window) to normalized device space
        # inverse viewport transform
        x = 2 * xx / width - 1
        y = 2 * (height - yy) / height - 1
        z = 2 * zz - 1

        # inverse transformation
        X = np.array((x, y, z, 1))
        X = view_inv @ proj_inv @ X
        X /= X[-1]
        X = X[:3]

        #if not kuka_env.validate_grip_pos([X]):
        #    continue

        # pick position above tray
        pos_gripper_up = [X[0], X[1], X[2] + LIFT_OFFS]
        pos_gripper_down = X

        # orientation of gripper (pointing down)
        yaw = math.pi * (aa+90) / 180
        orient_gripper = p.getQuaternionFromEuler([0, -math.pi, yaw])

        # move arm to this starting position, with gripper open
        success = kuka_env.move_arm_to_pose(desired_pos_gripper=pos_gripper_up,
                                            desired_orient_gripper=orient_gripper,
                                            desired_finger_angle=0.3)
        print("Start pos:", success)
        for _ in range(100):
            p.stepSimulation()

        # move down into tray
        success = kuka_env.move_arm_to_pose(desired_pos_gripper=pos_gripper_down,
                                            desired_orient_gripper=orient_gripper,
                                            desired_finger_angle=0.3)
        print("Grasp open pos:", success)
        for _ in range(100):
            p.stepSimulation()

        # grasp!
        pos_gripper = pos_gripper_down
        pos_gripper[2] -= 0.20
        success = kuka_env.move_arm_to_pose(desired_pos_gripper=pos_gripper,
                                            desired_orient_gripper=orient_gripper,
                                            desired_finger_angle=0)
        print("Grasp closed pos:", success)
        for _ in range(100):
            p.stepSimulation()

        # lift!
        success = kuka_env.move_arm_to_pose(desired_pos_gripper=pos_gripper_up,
                                            desired_orient_gripper=orient_gripper,
                                            desired_finger_angle=0)
        print("Lift pos:", success)
        for _ in range(100):
            p.stepSimulation()

        # check if object is lifted for x steps
        success = kuka_env.validate_grasp()
        print("GRASP:", success)

        idx = 0 if success else 1
        if i == 0:
            top_1[idx] += 1
        elif i <= 1:
            top_2[idx] += 1
        elif i <= 2:
            top_3[idx] += 1
        print("1st try success:", top_1, top_1[0]/(1e-10+top_1[0]+top_1[1]))
        print("2nd try success:", top_2, top_2[0]/(1e-10+top_2[0]+top_2[1]))
        print("3rd try success:", top_3, top_3[0]/(1e-10+top_3[0]+top_3[1]))
        if i == 2 or success:
            break

p.disconnect()
