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





  def move(self, coords):
    
    endEffectorLinkIndex = self.pepper.link_dict["r_hand"].getIndex()
    ik = p.calculateInverseKinematics(self.bodyUniqueId, endEffectorLinkIndex, coords)
    self.simulation_manager.stepSimulation(self.client)
    for joint_parameter in self.joint_parameters:
                 self.pepper.setAngles(
                     joint_parameter[1],
                     ik, 1.0)
    self.simulation_manager.stepSimulation(self.client)

   #p.setJointMotorControlArray(self.bodyUniqueId, list(range(len(ik))), p.POSITION_CONTROL, targetPositions=ik)


if __name__ == "__main__":
 pepper_sim = PepperSimulator()
 pepper_sim.move([])


