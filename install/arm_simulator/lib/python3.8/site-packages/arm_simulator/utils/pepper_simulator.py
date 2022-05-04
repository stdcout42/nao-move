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
    print(f'translation: {self.pepper.getLinkPosition("r_hand")[0]}')

  def move(self, coords):
    endEffectorLinkIndex = self.pepper.link_dict["r_hand"].getIndex()
    p.stepSimulation()
    print(f'eeli r_hand: {endEffectorLinkIndex}')
    ik = p.calculateInverseKinematics(self.bodyUniqueId, endEffectorLinkIndex, coords)
    p.setJointMotorControlArray(self.bodyUniqueId, list(range(endEffectorLinkIndex-1)), p.POSITION_CONTROL, targetPositions=ik)

if __name__ == "__main__":
  main()
