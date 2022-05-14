#!/usr/bin/env python
# coding: utf-8

import cv2
import time
import math
import pybullet as p
from qibullet import SimulationManager
from qibullet import PepperVirtual
from qibullet import Camera

class PepperSimulator(object):
  def __init__(self):
    self.simulation_manager = SimulationManager()
    self.client = self.simulation_manager.launchSimulation(gui=True)
    self.pepper = self.simulation_manager.spawnPepper(self.client, spawn_ground_plane=True, quaternion=p.getQuaternionFromEuler( [0, 0,-math.pi/2]) )
    self.bodyUniqueId = self.pepper.getRobotModel()
    self.joints = list(self.pepper.joint_dict.keys())
    #p.setRealTimeSimulation(False)


  def move(self, coords, joint_to_move='l_hand'):
   endEffectorLinkIndex = self.pepper.link_dict['l_hand'].getIndex()
   ik = p.calculateInverseKinematics(self.bodyUniqueId, endEffectorLinkIndex, coords)
   joints_to_control = ['lshoulder', 'lelbow']
   for i, joint in enumerate(self.joints):
     for j in joints_to_control:
       if j in joint.lower():
         print(f'{joint} {ik[i]}')
         self.pepper.setAngles(joint, ik[i], 1)
         #self.simulation_manager.stepSimulation(self.client)

if __name__ == "__main__":
 pepper_sim = PepperSimulator()
 pepper_sim.move([])


