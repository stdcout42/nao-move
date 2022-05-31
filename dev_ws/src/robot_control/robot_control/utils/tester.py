import numpy as np
from os.path import join
from datetime import datetime
from enum import Enum, auto

class AutoName(Enum):
  def _generate_next_value_(name, start, count, last_values): #pylint: disable=no-self-argument
    return name

class Shape(AutoName):
  CIRCLE = auto()

class Tester():
  EXP_DIR = 'src/robot_control/robot_control/experiments'
  shape_to_test = Shape.CIRCLE

  def __init__(self, simulator):
    self.simulator = simulator


  def get_filename(self, shape, key_press=False):
    key_press = 'KEY_' if key_press else ''
    date =  datetime.now().strftime('%m_%d-%H_%M_%S')
    return f'{key_press}{shape.name}_{date}'

  def save_trajectory(self, trajectory, shape=None, key_press=False):
    if not shape:
      shape = self.shape_to_test

    np.save(join(self.EXP_DIR, self.get_filename(shape, key_press)), 
        np.array(trajectory))
  
  def draw_test_shape(self, shape=None):
    if not shape:
      shape = self.shape_to_test

    trajectory = self.get_shape_trajectory_from_file(shape)
    self.simulator.draw_trajectory(trajectory)

  def get_shape_trajectory_from_file(self, shape):
    return np.load(join(self.EXP_DIR, f'{shape.name}.npy')).tolist()
