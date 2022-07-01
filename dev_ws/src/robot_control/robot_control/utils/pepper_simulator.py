#!/usr/bin/env python
# coding: utf-8

import cv2
import time
import copy
import math
import pybullet as p
import pybullet_data
from qibullet import SimulationManager
from .enums import Obj, Shape
from .math_utils import *

L_FINGER21 = 'LFinger21_link'
L_HAND = 'l_hand'
L_WRIST = 'l_wrist'
DEFAULT_LINK = L_FINGER21
PAN_PATH = 'src/robot_control/robot_control/objects/dinnerware/pan_tefal.urdf'

class PepperSimulator():
  name = 'PEPPER'
  table = None
  init_table_pos = [0.3, -0.5, 0.15]
  ball = None
  init_ball_pos = [0.33, -0.31, 1]
  ref_ball = None
  tray = None
  init_tray_pos = [0.95, -0.25, 0]
  testing_started = False
  is_recording = False
  full_test = False
  last_coords = []
  test_shape_coords = []
  test_shape_lines = []
  user_test_movement = []
  ref_ball_path = 'src/robot_control/robot_control/objects/my_ball.urdf'
  shape_coord_index = 0
  curr_link = DEFAULT_LINK
  depth = -0.3 
  depth_is_fixed = False
  max_angle = 0.15
  STARTING_ORN = -math.pi/2
  INIT_HIP_POS = [0, -0.004135, 0.60196]
  two_arms_enabled = False

  def __init__(self):
    self.simulation_manager = SimulationManager()
    self.client = self.simulation_manager.launchSimulation(gui=True)
    self.pepper = self.simulation_manager.spawnPepper(self.client, spawn_ground_plane=True, 
        quaternion=p.getQuaternionFromEuler([0,0,self.STARTING_ORN]))
    self.pepper_id = self.pepper.getRobotModel()
    self.joints = list(self.pepper.joint_dict.keys())
    self.initial_link_pos = self.pepper.getLinkPosition(DEFAULT_LINK)[0]
    self.link_prev_pos = self.pepper.getLinkPosition(DEFAULT_LINK)[0]
    self.link_current_pos = self.pepper.getLinkPosition(DEFAULT_LINK)[0]
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setRealTimeSimulation(True)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

  def set_test_shape_coords(self, coords):
    self.test_shape_coords = coords

  def place_starting_shape_reference(self, shape):
    self.add_ref_ball(self.test_shape_coords[0])

  def add_ref_ball(self, pos):
    if self.ref_ball:
      p.removeBody(self.ref_ball)
    self.ref_ball = p.loadURDF(self.ref_ball_path, pos, globalScaling=0.8)

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
      

  def move_joint(self, coords, end_effector=DEFAULT_LINK, draw=False, color=[1,0,0]):
    endEffectorLinkIndex = self.pepper.link_dict[end_effector].getIndex()
    coords = self.adjust_coordinates(coords)
    coords = rotate_vector_zaxis(self.get_change_in_orn(), coords)
    coords = self.adjust_translation_base_coords(coords)
    self.last_coords = coords

    #print(f'Moving sim to x,y,z with z being height {coords}')
    ik = p.calculateInverseKinematics(self.pepper_id, endEffectorLinkIndex, coords)
    if self.two_arms_enabled:
      if end_effector == 'r_hand': 
        joints_to_control = ['rshoulder', 'relbow',]
      else:
        joints_to_control = ['lshoulder', 'lelbow',]
    else:
      joints_to_control = ['lshoulder', 'lelbow', 'hiproll', 'kneepitch', ]

    if self.testing_started:
      color = [182/255, 3/255, 252/255]

    for i, joint in enumerate(self.joints):
      for j in joints_to_control:
        if j.lower() in joint.lower():
         #print(f'{joint} {ik[i]}')
         angle = ik[i]
         if j.lower() in ('kneepitch','hippitch'):
           if math.fabs(angle) > self.max_angle: 
             angle = self.max_angle if angle > 0 else -self.max_angle

         self.pepper.setAngles(joint, angle, 1)
    if not self.two_arms_enabled and draw:
      self.link_current_pos = self.pepper.getLinkPosition(end_effector)[0]
      p.addUserDebugLine(self.link_prev_pos, self.link_current_pos, color, 2, 40)
      self.link_prev_pos = self.link_current_pos
    elif end_effector != 'r_hand':
      self.link_prev_pos = self.pepper.getLinkPosition(end_effector)[0]

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
    coords[0] += self.INIT_HIP_POS[0]
    coords[1] += self.INIT_HIP_POS[1]
    coords[2] += self.INIT_HIP_POS[2]
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

  def draw_coordinates(self, coords, color=[0.1, 0.1, 0.8], width=1):
    for i, coord in enumerate(coords):
      if i == len(coords)-1: 
        self.test_shape_lines.append(p.addUserDebugLine(coord, coords[0],
          color, width, 0))
      else:
        self.test_shape_lines.append(p.addUserDebugLine(coord, coords[i+1], color, width, 0))

  def spawn_obj(self, obj_str):
    obj = get_obj_from_str(obj_str)
    if obj == Obj.BALL:
      if not self.ball:
        self.ball = p.loadURDF('soccerball.urdf',
            basePosition=self.init_ball_pos,  globalScaling=0.2)
        p.changeDynamics(self.ball, -1, mass=0.1)
      else:
        p.removeBody(self.ball)
        self.ball = None
    elif obj == Obj.TABLE:
      if not self.table:
        self.table = p.loadURDF("table_square/table_square.urdf", 
            basePosition=self.init_table_pos)
        self.draw_ground_marker()
      else:
        p.removeBody(self.table)
        self.table = None
    elif obj == Obj.TRAY:
      if not self.tray:
        self.tray = p.loadURDF("tray/tray.urdf",
            basePosition=self.init_tray_pos)
      else:
        p.removeBody(self.tray)
        self.tray = None

  def spawn_cookware(self):
    if self.table:
      p.removeBody(self.table)
    if self.tray:
      p.removeBody(self.tray)

    self.table = p.loadURDF("table_square/table_square.urdf", basePosition=[0.3, -0.5,0])
    self.pan = p.loadURDF(PAN_PATH, basePosition=[.15,-.35,.65])
  
  def set_init_obj_position(self, obj_str, pos):
    obj = get_obj_from_str(obj_str)
    if obj == Obj.BALL:
      if self.ball:
        p.removeBody(self.ball) 
        p.ball = None
      self.init_ball_pos = pos
    elif obj == Obj.TABLE:
      if self.table:
        p.removeBody(self.table)
        self.table = None
      self.init_table_pos = pos
    elif obj == Obj.TRAY:
      if self.tray:
        p.removeBody(self.tray)
        self.tray = None
      self.init_tray_pos = pos
 
  def move_ref_and_measure_on_rdy(self):
    if not self.testing_started:
      if self.full_test and not self.is_recording: 
        return True

      self.testing_started = self.is_link_within_ref_dist()
      if self.testing_started:
        self.user_test_movement.append(self.pepper.getLinkPosition(self.curr_link)[0])
      return True

    self.shape_coord_index += 1
    if self.shape_coord_index < len(self.test_shape_coords):
      p.removeBody(self.ref_ball)
      self.ref_ball = p.loadURDF(self.ref_ball_path, 
          self.test_shape_coords[self.shape_coord_index], globalScaling=0.8)
      self.user_test_movement.append(self.pepper.getLinkPosition(self.curr_link)[0])
      return True 
    self.shape_coord_index = 0
    self.testing_started = False
    return False

  def set_depth(self, turn_on, depth=None):
    if turn_on:
      self.depth_is_fixed = False 
    else:
      self.depth_is_fixed = True 
      if depth is not None:
        self.depth = depth

  def set_max_angle(self, angle):
    self.max_angle = angle

  def is_link_within_ref_dist(self, lim=0.1):
    link_pos = self.pepper.getLinkPosition(self.curr_link)[0]
    ref_ball_pos = p.getBasePositionAndOrientation(self.ref_ball)[0]
    return get_dist(link_pos, ref_ball_pos) < lim

  def remove_all_debug(self):
    p.removeAllUserDebugItems()
    if self.ref_ball:
      p.removeBody(self.ref_ball)
      self.ref_ball = None

  def get_change_in_orn(self):
    return \
        p.getEulerFromQuaternion(self.pepper.getLinkPosition('base_footprint')[1])[2] \
        - self.STARTING_ORN

  def get_link_pos(self):
    return self.pepper.getLinkPosition(self.curr_link)[0]

  def get_last_coords(self):
    return self.last_coords

  def reset_base(self):
    p.resetBasePositionAndOrientation(self.pepper_id, [0,0,0], 
        p.getQuaternionFromEuler([0,0,self.STARTING_ORN]))
  
  def set_objs_close(self):
    self.init_table_pos = [0.3, -0.5, 0.15]
    self.init_ball_pos = [0.33, -0.31, 1]
    self.init_tray_pos = [0.95, -0.25, 0]

  def set_objs_far(self):
    self.init_table_pos = [-1, 2, 0.15]
    self.init_ball_pos = [-1.03, 1.81, 1]
    self.init_tray_pos = [-1.65, 1.75, 0]


  def adjust_translation_base_coords(self, coords):
    delta = np.array(self.pepper.getLinkPosition('base_footprint')[0])
    return (np.array(coords) + delta).tolist()
  
  def draw_ground_marker(self):
    pos = np.array(self.init_table_pos) + np.array([0.3, -0.5, 0.15])
    pos[2]+= .3
    self.draw_coordinates(get_coords_ground_circle(center=pos), color=(1,0,0), width=3)

  def set_base_to_far_table(self):
    pos = np.array(self.init_table_pos) + np.array([0.3, -0.5, -0.15]).tolist()
    p.resetBasePositionAndOrientation(self.pepper_id, pos, p.getQuaternionFromEuler([0,0,math.pi/2]))


def get_obj_from_str(obj_str):
  for obj in list(Obj):
    if obj.name.lower() == obj_str:
      return obj
  return None

