import pybullet as p
import numpy as np
from pybullet_envs.bullet import kuka
import pybullet_data
import math
import util as u
import random
import glob
import os

class KukaEnv(object):

    def __init__(self, gui, urdf_dir):
        self.sim_steps = 0
        self.frame_num = 0
        self.obj_ids = []
        self.urdf_paths = glob.glob(f"{urdf_dir}/*/*.urdf")
        print(self.urdf_paths)

        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        # load table and arm
        p.loadURDF(pybullet_data.getDataPath()+"/table/table.urdf", 0.5,0.,-0.82, 0,0,0,1)
        self.kuka = kuka.Kuka(urdfRootPath=pybullet_data.getDataPath())
        # remove tray (part of kuka env)
        p.removeBody(self.kuka.trayUid)

        # activate apple physics
        p.setGravity(0, 0, -9.8)

        #p.enableJointForceTorqueSensor(self.kuka.kukaUid, 8)
        #p.enableJointForceTorqueSensor(self.kuka.kukaUid, 11)

        self.left_finger_index = 8
        self.left_fingertip_index = 10
        self.right_finger_index = 11
        self.right_fingertip_index = 13


    def drop_objects(self, num_objects):
        """
        Drop 'num_objects' objects onto surface. Objects are randomly picked from given urdf directory.
        """
        for obj_uid in self.obj_ids:
            p.removeBody(obj_uid)
        self.obj_ids.clear()

        # drop semi-random objects into tray
        for _ in range(num_objects):
            #random_obj_id = random.randint(0, 9)
            #urdf_filename = "%s/%04d/%04d.urdf" % (obj_urdf_dir, random_obj_id, random_obj_id)
            urdf_filename = np.random.choice(self.urdf_paths)
            print("Dropping:", os.path.basename(urdf_filename))

            # drop block fixed x distance from base of arm (0.51)
            # across width of tray (-0.1, 0.3) and from fixed height (0.2)
            block_pos = [u.random_in(0.51, 0.66), u.random_in(-0.1, 0.3), -0.1]

            # drop with random yaw rotation
            block_angle = random.random() * math.pi
            block_orient = p.getQuaternionFromEuler([0, 0, block_angle])

            obj_uid = p.loadURDF(urdf_filename, basePosition=block_pos, baseOrientation=block_orient, flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
            self.obj_ids.append(obj_uid)

            # let object fall and settle to be clear of next
            for _ in range(500):
                p.stepSimulation()


    def validate_grasp(self, nsteps=500, zlim=0.01):
        """
        Check if one of the objects is lifted up in the air.
        Object needs to stay 'nsteps' above 'zlim' height threshold.
        """
        for obj_id in self.obj_ids:
            X = p.getBasePositionAndOrientation(obj_id)[0]

            # object needs to stay lifted up for many steps
            res = True
            for _ in range(nsteps):
                p.stepSimulation()
                if X[2] < zlim: # one failure is enough to break
                    res = False
                    break
            if res: # return if we have success
                return True

        return False


    def validate_grip_pos(self, targets, thres=1.0):
        """
        Check if 3D positions given by 'targets' are within threshold 'thres' of object positions.
        Use to check validity of image/perspective/world transformation.
        """
        assert len(targets) == len(self.obj_ids)
        res = True

        for i in range(len(targets)):
            X_t = targets[i]
            X = p.getBasePositionAndOrientation(self.obj_ids[i])[0]
            diff = np.linalg.norm(X_t-X)
            #print("++")
            #print("BODY:", X)
            #print("GRASP:", X_t)
            #print("DIFF:", diff)
            #print("++")
            if diff > thres:
                res = False
        return res

    def get_joint_info(self):
        num_joints = p.getNumJoints(self.kuka.kukaUid)
        for i in range(num_joints):
            info = p.getJointInfo(self.kuka.kukaUid, i)
            print(info[0], info[-5], info[-3])


    # hand rolled IK move of arm without constraints, dx/dy/dz limits in kuka class
    def move_arm_to_pose(self, desired_pos_gripper, desired_orient_gripper,
                         desired_finger_angle, max_steps=1000):
        """
        Move robot gripper to given position, orientation and opening.
        """
        # x range (0.5, 0.7)
        # y range (-0.15, 0.25)
        # z range (0.15, 0.4)    # grasp at 0.1 to 0.15 (0.125)?

        def set_joints(from_index, to_index):
            # set motor control for them
            for i in range(from_index, to_index + 1):
                info = p.getJointInfo(self.kuka.kukaUid, i)
                p.setJointMotorControl2(bodyUniqueId=self.kuka.kukaUid,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=joint_poses[i],
                                        targetVelocity=0,
                                        force=self.kuka.maxForce,
                                        maxVelocity=self.kuka.maxVelocity,
                                        positionGain=0.3,
                                        velocityGain=1)


        # TODO: for some reason tool TCP is not at tool tip
        #desired_pos_gripper[2] += 0.19 # TODO: left finger tip - base
        ## tool centering compensation
        #eul = p.getEulerFromQuaternion(desired_orient_gripper)
        #desired_pos_gripper[0] += -0.02 * np.sin(eul[2])
        #desired_pos_gripper[1] += -0.02 * np.cos(eul[2])

        desired_pos_gripper = np.asarray(desired_pos_gripper)

        #self.get_joint_info()

        # calculate TCP and shift desired position from TCP to base 
        gripper_state = p.getLinkState(self.kuka.kukaUid, self.kuka.kukaGripperIndex)
        left_fingertip_state = p.getLinkState(self.kuka.kukaUid, self.left_fingertip_index)
        right_fingertip_state = p.getLinkState(self.kuka.kukaUid, self.right_fingertip_index)
        tcp = 1/2 * (np.asarray(left_fingertip_state[0]) + np.asarray(right_fingertip_state[0]))
        tcp_offs = tcp - np.asarray(gripper_state[0])

        desired_pos_gripper -= tcp_offs # shift desired pos
        #desired_pos_gripper[2] -= 0.01

        steps = 0
        while True:
            steps += 1
            if steps > max_steps:
                return False

            gripper_state = p.getLinkState(self.kuka.kukaUid, self.kuka.kukaGripperIndex)
            actual_pos_gripper = gripper_state[0]
            actual_orient_gripper = gripper_state[1]

            # calculate euclidean distance between desired and actual
            # gripper positions
            pos_diff = np.linalg.norm(np.array(desired_pos_gripper)-np.array(actual_pos_gripper))

            # qaternions align when their w component is near 1.0
            diff_quant = p.getDifferenceQuaternion(desired_orient_gripper, actual_orient_gripper)
            diff_quant_w = diff_quant[3]

            diff_eul = p.getEulerFromQuaternion(diff_quant)
            diff_eul_z = diff_eul[2]

            # finger?
            left_side_finger_angle = -p.getJointState(self.kuka.kukaUid, self.left_finger_index)[0]
            right_side_finger_angle = p.getJointState(self.kuka.kukaUid, self.right_finger_index)[0]
            left_side_diff = left_side_finger_angle - desired_finger_angle
            right_side_diff = right_side_finger_angle - desired_finger_angle

            # get closure force in x
            #forces = p.getJointState(self.kuka.kukaUid, self.left_finger_index)[2]
            #force_x = forces[0]

            # if pos, orient and fingers look good, we are done!
            # diff_quant_w > 0.9999 
            if (pos_diff < 0.001 and diff_eul_z < 0.0001 # and (force_x < 0 or force_x > 5)):
                    and left_side_diff < 0.01 and right_side_diff < 0.01):
                return True

            # use IK to calculate target joint positions
            # (ignore null space)
            joint_poses = p.calculateInverseKinematics(self.kuka.kukaUid, self.kuka.kukaGripperIndex,
                                                       desired_pos_gripper,
                                                       desired_orient_gripper,
                                                       self.kuka.ll, self.kuka.ul, self.kuka.jr, self.kuka.rp)
            set_joints(0, self.kuka.kukaGripperIndex)

            # gripper fingers
            p.setJointMotorControl2(self.kuka.kukaUid, 8, p.POSITION_CONTROL,
                                    targetPosition=-desired_finger_angle, force=self.kuka.fingerAForce)
            p.setJointMotorControl2(self.kuka.kukaUid, 11, p.POSITION_CONTROL,
                                    targetPosition=desired_finger_angle,force=self.kuka.fingerBForce)
            p.setJointMotorControl2(self.kuka.kukaUid, 10, p.POSITION_CONTROL,
                                    targetPosition=0, force=self.kuka.fingerTipForce)
            p.setJointMotorControl2(self.kuka.kukaUid, 13, p.POSITION_CONTROL,
                                    targetPosition=0, force=self.kuka.fingerTipForce)

            # step sim!
            p.stepSimulation()

        for _ in range(500):
            p.stepSimulation()
