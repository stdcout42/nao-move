import numpy as np
from os.path import join
from datetime import datetime
from enum import Enum, auto
from .math_utils import * 

class AutoName(Enum):
  def _generate_next_value_(name, start, count, last_values): #pylint: disable=no-self-argument
    return name

class Shape(AutoName):
  CIRCLE = auto()
  SQUARE = auto()
  TRIANGLE = auto()
  HORIZONTAL_LINE = auto()
  VERTICAL_LINE = auto()
  DIAGONAL_LINE = auto()

class Tester():
  EXP_DIR = 'src/robot_control/robot_control/experiments'
  shape_to_test = Shape.CIRCLE
  test_shape_coords = []
  subject_name = ''
  shape_modification = '' 

  def __init__(self, simulator):
    self.simulator = simulator

  def set_test_shape(self, shape, modification=''):
    l_hand_pos = self.simulator.initial_l_hand_pos
    self.shape_modification = modification
    self.shape_to_test = shape
    if shape == Shape.TRIANGLE:
      self.test_shape_coords = get_vertices_triangle(center=l_hand_pos)
    elif shape == Shape.SQUARE:
      self.test_shape_coords = get_vertices_square(center=l_hand_pos)
    elif shape == Shape.CIRCLE:
      self.test_shape_coords = get_circle_coords(center=l_hand_pos)

    if modification:
      self.test_shape_coords = get_modified_trajectory(self.test_shape_coords, modification)

   

  def get_filename(self, shape, key_press=False):
    key_press = 'KEY_' if key_press else ''
    name = f'{self.subject_name}_' if self.subject_name else ''
    mod = f'{self.shape_modification}_' if self.shape_modification else''

    date =  datetime.now().strftime('%m_%d-%H_%M_%S')
    return f'{key_press}{name}{shape.name}{mod}_{self.simulator.name}_{date}'

  def save_trajectory(self, trajectory, shape=None, key_press=False):
    if not shape:
      shape = self.shape_to_test

    adjusted_trajectory = list(map( 
      self.simulator.adjust_coordinates, trajectory))

    np.save(join(self.EXP_DIR, self.get_filename(shape, key_press)), 
        np.array(adjusted_trajectory))
  
  def draw_test_shape(self, shape=None, modification=None):
    if not shape:
      shape = self.shape_to_test
    self.set_test_shape(shape, modification)
    self.simulator.draw_coordinates(self.test_shape_coords)

  def get_shape_trajectory_from_file(self, shape):
    return np.load(join(self.EXP_DIR, f'{shape.name}.npy')).tolist()
