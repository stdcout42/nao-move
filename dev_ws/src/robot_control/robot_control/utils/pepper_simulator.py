#!/usr/bin/env python
# coding: utf-8

import cv2
import time
import copy
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
    self.l_hand_prev_pos = self.pepper.getLinkPosition('l_hand')[0]
    self.l_hand_current_pos = self.pepper.getLinkPosition('l_hand')[0]
  
  def move(self, direction, speed=1):
    y_speed = 0
    x_speed = 0
    if direction == 'FORWARD':
      y_speed = 0.1
    elif direction == 'BACK':
      y_speed = -0.1
    elif direction == 'RIGHT':
      x_speed = -0.1
    elif direction == 'LEFT':
      x_speed = 0.1

    self.pepper.move(y_speed*speed, x_speed*speed, 0)
      

  def move_joint(self, coords, end_effector='l_hand', draw=False):
    endEffectorLinkIndex = self.pepper.link_dict['l_hand'].getIndex()
    coords = self.adjust_coordinates(coords)
    #print(f'Moving sim to x,y,z with z being height {coords}')
    ik = p.calculateInverseKinematics(self.bodyUniqueId, endEffectorLinkIndex, coords)
    joints_to_control = ['lshoulder', 'lelbow', 'hiproll', 'kneepitch', ]

    max_angle = 0.15
    for i, joint in enumerate(self.joints):
      for j in joints_to_control:
        if j.lower() in joint.lower():
         #print(f'{joint} {ik[i]}')
         angle = ik[i]
         if j.lower() in ('kneepitch','hippitch'):
           if math.fabs(angle) > max_angle: 
             angle = max_angle if angle > 0 else -max_angle

         self.pepper.setAngles(joint, angle, 1)
    if draw:
      self.l_hand_current_pos = self.pepper.getLinkPosition('l_hand')[0]
      p.addUserDebugLine(self.l_hand_prev_pos, self.l_hand_current_pos, [1,0,0], 2, 15)
      self.l_hand_prev_pos = self.l_hand_current_pos
    else: self.l_hand_prev_pos = self.pepper.getLinkPosition('l_hand')[0]

  """
  Func: adjust_coordinates
  coords: [x,y,z] with y being depth, z being 
          height, received from mediapipe world
          received from mediapipe world coords

  returns: coordinates adjusted for pepper's imulation
          in PyBullet
  """
  def adjust_coordinates(self, coords):
    coords = copy.copy(coords)
    coords[2] *= -1
    coords[0] += self.pepper.getLinkPosition('Hip')[0][0]
    coords[1] += self.pepper.getLinkPosition('Hip')[0][1]
    coords[2] += self.pepper.getLinkPosition('Hip')[0][2]
    return coords

  def draw_trajectory(self, trajectory, color=[0.5,1,0], duration=60):
    for i, coord in enumerate(trajectory):
      if i + 1 == len(trajectory): break
      coord_from = coord
      coord_to = trajectory[i+1]
      coord_from = self.adjust_coordinates(coord_from)
      coord_to = self.adjust_coordinates(coord_to)
      p.addUserDebugLine(coord_from, coord_to, color, 2, duration)

  def add_text(self, text, pos, color):
    pos = self.adjust_coordinates(pos)
    pos[0] += 0.2
    pos[2] += 0.2
    p.addUserDebugText(text, pos, color, 2, 60)

  def remove_all_debug(self):
    p.removeAllUserDebugItems()
