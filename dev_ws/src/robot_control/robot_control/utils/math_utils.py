import math
import copy 
import numpy as np

def get_modified_trajectory(trajectory, modification, angle=math.pi/2):
  trajectory_copy = copy.deepcopy(trajectory)
  if modification in ('roll', 'pitch', 'yaw'):
    return get_rotated_trajectory(trajectory, modification)
  return get_translated_trajectory(trajectory_copy, modification)
  
def get_rotated_trajectory(trajectory, modification, angle=math.pi/4):
  centroid = get_centroid(trajectory)
  if modification == 'roll':
    trajectory =  list(map(lambda v: rotate_vector_xaxis(angle, v), trajectory))
  elif modification == 'pitch':
    trajectory = list(map(lambda v: rotate_vector_yaxis(-angle, v), trajectory))
  else:
    trajectory = list(map(lambda v: rotate_vector_zaxis(-angle, v), trajectory))
  new_centroid = get_centroid(trajectory)
  if new_centroid.tolist() != centroid.tolist():
    trajectory = get_resetted_centroid_trajectory(trajectory, centroid, new_centroid)
  return trajectory

def get_translated_trajectory(trajectory, modification):
  centroid = get_centroid(trajectory)
  dx = dy = dz = 0
  c = 1.0
  if modification == 'up':
    dz = 0.05
  elif modification == 'down':
    dz = -0.05
  elif modification == 'right':
    dx = 0.05 
  elif modification == 'left':
    dx = -0.05
  elif modification == 'bigger':
    c = 1.1
  elif modification == 'smaller':
    c = 0.9
  trajectory = list(map(lambda v: [v[0]*c+dx, v[1]+dy, v[2]*c-dz], 
    trajectory))
  new_centroid = get_centroid(trajectory)
  if modification in ('bigger', 'smaller')\
      and new_centroid.tolist() != centroid.tolist():
    trajectory = get_resetted_centroid_trajectory(trajectory, centroid, new_centroid)
  return trajectory


def get_resetted_centroid_trajectory(trajectory, old_centroid, new_centroid):
    d_centroid = old_centroid - new_centroid
    return list(map(lambda v: (np.array(v) + d_centroid).tolist(), trajectory))


def get_centroid(coords):
  return np.add.reduce(np.array(coords)) / len(coords)


# matrices from: https://en.wikipedia.org/wiki/Rotation_matrix
def rotate_vector_xaxis(angle, vector):
  rot_matrix = np.array([
    [1, 0, 0],
    [0, math.cos(angle), -math.sin(angle)],
    [0, math.sin(angle), math.cos(angle)]
    ])

  t_x = np.matmul(rot_matrix, np.array(vector))
  return [t_x[0], t_x[1], t_x[2]]

# matrices from: https://en.wikipedia.org/wiki/Rotation_matrix
def rotate_vector_yaxis(angle, vector):
  rot_matrix = np.array([
    [math.cos(angle), 0, math.sin(angle)],
    [0, 1, 0], 
    [-math.sin(angle), 0, math.cos(angle)]
    ])

  t_x = np.matmul(rot_matrix, np.array(vector))
  return [t_x[0], t_x[1], t_x[2]]

# matrices from: https://en.wikipedia.org/wiki/Rotation_matrix
def rotate_vector_zaxis(angle, vector):
  rot_matrix = np.array([
    [math.cos(angle), -math.sin(angle), 0],
    [math.sin(angle), math.cos(angle), 0], 
    [0, 0, 1]
    ])

  t_x = np.matmul(rot_matrix, np.array(vector))
  return [t_x[0], t_x[1], t_x[2]]

def get_vertices_square(side_length = 0.3, center=(0,0,0)):
  half_length= side_length / 2.0
  top_left = [center[0] - half_length, center[1], center[2] + half_length]
  bottom_left = [center[0] - half_length, center[1], center[2] - half_length]
  top_right = [center[0] + half_length, center[1], center[2] + half_length]
  bottom_right = [center[0] + half_length, center[1], center[2] - half_length]
  return [top_left, bottom_left, bottom_right, top_right]
 
def get_coordinates_square(vertices):
  # vertices: (top_left, bottom_left, bottom_right, top_right)
  coords = []
  step_sz = 0.01

  for v_i, vertex in enumerate(vertices):
    curr = vertex.copy()
    if v_i == 0: #top_left
      while curr[2] > vertices[v_i+1][2]:
        curr[2] -= step_sz
        coords.append([curr[0], curr[1], curr[2]])
    elif v_i == 1: # bot_left
      while curr[0] < vertices[v_i+1][0]:
        curr[0] += step_sz
        coords.append([curr[0], curr[1], curr[2]])
    elif v_i == 2: # bot_right 
      while curr[2] < vertices[v_i+1][2]:
        curr[2] += step_sz
        coords.append([curr[0], curr[1], curr[2]])
    else: # top_right
      while curr[0] > vertices[0][0]:
        curr[0] -= step_sz
        coords.append([curr[0], curr[1], curr[2]])
  return coords
  
def get_coordinates_circle(radius=0.15, center=(0,0,0)):
  coords = []
  for t in range (0, 70): 
    coords.append([radius*math.cos(t/10)+center[0], center[1], radius*math.sin(t/10)+ center[2]])
  return coords

def get_coords_ground_circle(radius=0.15, center=(0,0,0)):
  coords = []
  for t in range (0, 70): 
    coords.append([radius*math.cos(t/10)+center[0], radius*math.sin(t/10)+center[1], center[2]])
  return coords


def get_coordinates_triangle(vertices):
  # vertices top, left, bottom
  coords = []
  step_sz = 0.025

  for v_i, vertex in enumerate(vertices):
    curr = vertex.copy()
    step = 1
    if v_i == 0: # top
      while curr[2] > vertices[v_i+1][2]:
        curr = get_coord_on_line_with_step(step_sz*step,  vertex, vertices[v_i+1])
        coords.append(curr)
        step += 1
    elif v_i == 1: #bot_left
      while curr[0] < vertices[v_i+1][0]:
        curr = get_coord_on_line_with_step(step_sz*step,  vertex, vertices[v_i+1])
        coords.append(curr)
        step += 1
    else: # bot_right
      while curr[2] < vertices[0][2]:
        curr = get_coord_on_line_with_step(step_sz*step,  vertex, vertices[0])
        coords.append(curr)
        step += 1
  return coords

def get_vertices_triangle(side_length=0.35, center=(0,0,0)):
  """ 
  returns 3D x,y,z vertex coords of an
  equilateral triangle with
  center being the origin
  ex return: ([-1,2,3], [1,3,4], [2,3,4]) 
  """
  height = math.sin(np.deg2rad(60))*side_length
  left = [center[0] - side_length/2.0, center[1], center[2] - height/2.0]
  right = [center[0] + side_length/2.0, center[1], center[2] - height/2.0]
  top = [center[0], center[1], center[2] + height/2.0]

  return [top, left, right]

def get_coord_on_line_with_step(step_sz, v_from, v_to):
  v_from = np.array(v_from)
  v_to = np.array(v_to)
  return (v_from + step_sz*(v_to - v_from)).tolist()

def get_dist(v_a, v_b):
  v_a = np.array(v_a)
  v_b = np.array(v_b)
  return np.linalg.norm(v_a-v_b)
