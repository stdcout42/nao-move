import numpy as np
from os.path import join, exists
from os import mkdir
from datetime import datetime
from enum import Enum, auto
from .math_utils import * 
from .enums import Shape

class AutoName(Enum):
  def _generate_next_value_(name, start, count, last_values): #pylint: disable=no-self-argument
    return name

class Tester():
  EXP_DIR = 'src/robot_control/robot_control/experiments'
  shape_to_test = Shape.CIRCLE
  test_shape_coords = []
  subject_name = ''
  shape_modification = '' 

  def __init__(self, simulator):
    self.simulator = simulator

  def set_test_shape(self, shape, modification=''):
    link_pos = self.simulator.initial_link_pos
    self.shape_modification = modification
    self.shape_to_test = shape
    coords = []
    if shape == Shape.TRIANGLE:
      coords = get_vertices_triangle(center=link_pos)
    elif shape == Shape.SQUARE:
      coords = get_coordinates_square(get_vertices_square(center=link_pos))
    elif shape == Shape.CIRCLE:
      coords = get_circle_coords(center=link_pos)
    if modification:
      coords = get_modified_trajectory(self.test_shape_coords, modification)

    self.test_shape_coords = coords
    self.simulator.set_test_shape_coords(coords)

  def get_filename(self, shape, key_press=False, template=False, test=False):
    key_press = 'KEY_' if key_press else ''
    name = f'{self.subject_name}_' if self.subject_name else ''
    mod = f'{self.shape_modification}_' if self.shape_modification else''
    template = 'TEMPLATE_' if template else ''
    angle = ''
    depth_fixed = ''
    if test and not template:
      angle = f'{self.simulator.max_angle}rad_'
      depth_fixed = f'd{self.simulator.depth}_' if self.simulator.depth_is_fixed else ''

    date =  datetime.now().strftime('%m_%d_%H_%M_%S')
    return f'{key_press}{name}{template}{shape.name}_{angle}{depth_fixed}{mod}{self.simulator.name}_{date}'

  def save_trajectory(self, trajectory, shape=None, key_press=False, 
      test=False, template=False):
    if not shape:
      shape = self.shape_to_test

    if not test:
      trajectory = list(map(
        self.simulator.adjust_coordinates, trajectory))
   
    dir_name = datetime.now().strftime('%m_%d_%H')
    dir_path = join(self.EXP_DIR, dir_name)
    if not exists(dir_path):
        mkdir(dir_path)

    np.save(join(dir_path, self.get_filename(shape, key_press, template, test)), 
        np.array(trajectory))
   
  def draw_test_shape(self, shape=None, modification=None):
    if not shape:
      shape = self.shape_to_test
    self.set_test_shape(shape, modification)
    self.simulator.draw_coordinates(self.test_shape_coords)
    self.place_starting_shape_reference()

  def place_starting_shape_reference(self):
    self.simulator.place_starting_shape_reference(self.shape_to_test)
  
  def run_shape_test(self):
    is_testing = self.simulator.move_ref_and_measure_on_rdy()
    if not is_testing: # save files
      self.save_trajectory(self.simulator.user_test_movement, test=True)
      self.save_trajectory(self.simulator.test_shape_coords, test=True, template=True)
      self.simulator.user_test_movement.clear()
    return is_testing

  def get_shape_trajectory_from_file(self, shape):
    return np.load(join(self.EXP_DIR, f'{shape.name}.npy')).tolist()
