#!/usr/bin/env python
# coding: utf-8

import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import Camera
from PepperKinematics.inverse_kinematics import get_head_angles, get_arm_all_angles, get_torso_angles 
import pybullet as p

class PepperSimulator(object):
  def __init__(self):
    self.simulation_manager = SimulationManager()
    self.client = self.simulation_manager.launchSimulation(gui=True)
    self.pepper = self.simulation_manager.spawnPepper(self.client, spawn_ground_plane=True)
    self.bodyUniqueId = self.pepper.getRobotModel()
    print(f'initial position: {self.pepper.getLinkPosition("r_hand")[0]}')


  def move(self, coords):
   lelbow = [65.28, 27.67, 42.15]
   lwrist = [169.10, 313.44, 143.98]
   [t1, t2, t3, t4] = get_arm_all_angles(lelbow[0], lelbow[1], lelbow[2], lwrist[0], lwrist[1], lwrist[2], 'left')
   names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
   angles = [t1, t2, t3, t4]
   self.pepper.setAngles(names, angles, 1)
   print(f'translation: {self.pepper.getLinkPosition("r_hand")[0]}')
   endEffectorLinkIndex = self.pepper.link_dict["r_hand"].getIndex()
   #p.stepSimulation()
   self.simulation_manager.stepSimulation(self.client)
   #ik = p.calculateInverseKinematics(self.bodyUniqueId, endEffectorLinkIndex, coords)
   #p.setJointMotorControlArray(self.bodyUniqueId, list(range(len(ik))), p.POSITION_CONTROL, targetPositions=ik)
   time.sleep(5)

if __name__ == "__main__":
 pepper_sim = PepperSimulator()
 pepper_sim.move([])


