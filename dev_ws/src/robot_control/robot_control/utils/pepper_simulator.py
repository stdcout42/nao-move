#!/usr/bin/env python
# coding: utf-8

import cv2
import time
import copy
import math
import pybullet as p
import pybullet_data
from qibullet import SimulationManager
from .enums import Obj

class PepperSimulator():
  name = 'PEPPER'
  test_shape_lines = [] 
  table = None
  ball = None
  tray = None

  def __init__(self):
    self.simulation_manager = SimulationManager()
    self.client = self.simulation_manager.launchSimulation(gui=True)
    self.pepper = self.simulation_manager.spawnPepper(self.client, spawn_ground_plane=True, 
        quaternion=p.getQuaternionFromEuler( [0, 0,-math.pi/2]) )
    self.pepper_id = self.pepper.getRobotModel()
    self.joints = list(self.pepper.joint_dict.keys())
    self.initial_l_hand_pos = self.pepper.getLinkPosition('l_hand')[0]
    self.l_hand_prev_pos = self.pepper.getLinkPosition('l_hand')[0]
    self.l_hand_current_pos = self.pepper.getLinkPosition('l_hand')[0]
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setRealTimeSimulation(True)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

  def move(self, direction, speed=2):
    y_speed = 0
    x_speed = 0
    z_speed = 0
    if direction == 'FORWARD':
      y_speed = 0.1
    elif direction == 'BACK':
      y_speed = -0.1
    elif direction == 'RIGHT':
      x_speed = -0.1
    elif direction == 'LEFT':
      x_speed = 0.1
    elif direction == 'ROT_R':
      z_speed = -0.1
    elif direction == 'ROT_L':
      z_speed = 0.1
    self.pepper.move(y_speed*speed, x_speed*speed, z_speed*speed)
      

  def move_joint(self, coords, end_effector='l_hand', draw=False, color=[1,0,0]):
    endEffectorLinkIndex = self.pepper.link_dict['l_hand'].getIndex()
    coords = self.adjust_coordinates(coords)
    #print(f'Moving sim to x,y,z with z being height {coords}')
    ik = p.calculateInverseKinematics(self.pepper_id, endEffectorLinkIndex, coords)
    joints_to_control = ['lshoulder', 'lelbow', 'hiproll', 'kneepitch', ]

    max_angle = 0.10
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
      p.addUserDebugLine(self.l_hand_prev_pos, self.l_hand_current_pos, color, 2, 15)
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
    pos[0] += 0.05
    pos[2] += 0.16
    p.addUserDebugText(text, pos, color, 2, 60)

  def draw_coordinates(self, coords, color=[0.1, 0.1, 0.8]):
    for i, coord in enumerate(coords):
      if i == len(coords)-1: 
        self.test_shape_lines.append(p.addUserDebugLine(coord, coords[0],
          color, -1))
      else:
        self.test_shape_lines.append(p.addUserDebugLine(coord, coords[i+1], color, -1))

  def spawn_obj(self, obj_str):
    obj = self.get_obj_from_str(obj_str)
    if obj == Obj.BALL:
      if not self.ball:
        self.ball = p.loadURDF('soccerball.urdf',
            basePosition=[0.33, -0.31, 1], globalScaling=0.2)
        p.changeDynamics(self.ball, -1, mass=0.1)
      else:
        p.removeBody(self.ball)
        self.ball = None
    elif obj == Obj.TABLE:
      if not self.table:
        self.table = p.loadURDF("table_square/table_square.urdf", 
            basePosition=[0.3, -0.5, 0.15])
      else:
        p.removeBody(self.table)
        self.table = None
    elif obj == Obj.TRAY:
      if not self.tray:
        self.tray = p.loadURDF("tray/tray.urdf",
            basePosition=[0.95, -0.25, 0])
      else:
        p.removeBody(self.tray)
        self.tray = None

  def get_obj_from_str(self, obj_str):
    for obj in list(Obj):
      if obj.name.lower() == obj_str:
        return obj
  
  def remove_all_debug(self):
    p.removeAllUserDebugItems()
