#!/usr/bin/env python
# coding: utf-8

import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import Camera
import pybullet as p

class PepperSimulator(object):
  def __init__(self):
    self.simulation_manager = SimulationManager()
    self.client = self.simulation_manager.launchSimulation(gui=True)
    self.pepper = self.simulation_manager.spawnPepper(self.client, spawn_ground_plane=True)
    self.bodyUniqueId = self.pepper.getRobotModel()
    print(f'initial position: {self.pepper.getLinkPosition("r_wrist")[0]}')


  def move(self, coords):
   coords = [0.25, 0.1663, 1.5294]
   print(f'translation: {self.pepper.getLinkPosition("r_wrist")[0]}')
   endEffectorLinkIndex = self.pepper.link_dict["l_wrist"].getIndex()
   #p.stepSimulation()
   self.simulation_manager.stepSimulation(self.client)
   ik = p.calculateInverseKinematics(self.bodyUniqueId, endEffectorLinkIndex, coords)
   p.setJointMotorControlArray(self.bodyUniqueId, list(range(len(ik))), p.POSITION_CONTROL, targetPositions=ik)

if __name__ == "__main__":
 pepper_sim = PepperSimulator()
 pepper_sim.move([])


