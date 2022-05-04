import os
import sys
import errno
import pybullet as p
import pybullet_data
import time
from time import sleep
import math
from math import pi
import numpy as np

"""
Robot arm simulation set up based on a previous implementation
by sandbox-kimb-ura(TODO)
"""

class Simulator(object):
  def __init__(self):
    self.configure_pybullet()
    self.load_configure_arm()
    self.get_joints()
    self.set_consts()
    # adjust block mass
    p.changeDynamics(self.arm_id, self.joint_id["weight_joint"], mass=0.5)

  def configure_pybullet(self):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")
    p.setRealTimeSimulation(False)
    p.setGravity(0, 0, -10)

  def load_configure_arm(self):
    model_path = "src/arm_simulator/arm_simulator/objects/jaco_robotiq_object.urdf"
    self.arm_id = p.loadURDF(model_path, [0, 0, 0], useFixedBase=True)
    angle = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
    # variables specific to this arm
    self.eef_index = 8
    num_joints = 18
    self.rp = [-pi / 2, pi * 5 / 4, 0., pi / 2, 0., pi * 5 / 4, -pi / 2, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
    self.ll = [-pi] * num_joints
    self.ul = [pi] * num_joints
    self.jd = [0.000001] * num_joints
    self.jr = np.array(self.ll) - np.array(self.ul)
    self.ik_solver = p.IK_DLS

    for i in range(num_joints):
        p.resetJointState(self.arm_id, i, self.rp[i])
        p.enableJointForceTorqueSensor(self.arm_id, i)

  def get_joints(self):
    self.joint_id = {}
    for i in range(p.getNumJoints(self.arm_id)):
        jointInfo = p.getJointInfo(self.arm_id, i)
        self.joint_id[jointInfo[1].decode('UTF-8')] = jointInfo[0]
  def set_consts(self):
    self.controlled_joints = list(range(self.eef_index-1))
    self.num_controlled_joints = self.eef_index-1
    self.orn = p.getQuaternionFromEuler([math.pi / 2, math.pi / 2, math.pi / 2])
    self.targetVelocities = [0] * self.num_controlled_joints
    self.forces = [500] * self.num_controlled_joints
    self.positionGains = [0.03] * self.num_controlled_joints
    self.velocityGains = [1] * self.num_controlled_joints

  def move(self, coords):
    p.stepSimulation()
    jointPoses = p.calculateInverseKinematics(self.arm_id, self.eef_index, coords, self.orn,
                                                lowerLimits=self.ll,
                                                upperLimits=self.ul,
                                                jointRanges=self.jr,
                                                restPoses=self.rp,
                                                jointDamping=self.jd,
                                                solver=self.ik_solver)[:self.num_controlled_joints]
    p.setJointMotorControlArray(bodyIndex=self.arm_id,
                                jointIndices=self.controlled_joints,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=jointPoses,
                                targetVelocities=self.targetVelocities,
                                forces=self.forces,
                                positionGains=self.positionGains,
                                velocityGains=self.velocityGains)



