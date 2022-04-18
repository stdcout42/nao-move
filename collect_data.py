import os
import sys
import pandas as pd
import datetime 
import cv2
from os.path import join, exists

"""
Helper file to record and save training/validation/test images
Takes one arg: folder name. By default this
is set to 'data'. Examples: 'train', 'validation', 'test'
"""
def print_usage():
  print(f'Usage:\n\t python3 collect_data.py [output_folder]')

def create_filename():
  dt = datetime.datetime.now()
  year, month, day, hour, minute, second  = dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second
  return "{}{}{}_{}_{}_{}".format(year, month, day, hour, minute, second)


# Function is doing too much atm, needs to be modularized better
"""
Waits for key input and captures and saves TODO: elaborate
"""

class Data_Collector():
  keys_dict = {
      'l': 'left', 
      'r': 'right',
      'u': 'up',
      'd': 'down',
      's': 'stop', 
      'f': 'record_movement', 
      'y': 'yaw', 
      'b': 'big',
      'm': 'small', 
      'p': 'pitch', 
      'o': 'OK',
      'n': 'negative',
      't': 'replay_movement'
  }
  
  def __init__(self, output_dir):
    self.output_dir = output_dir
    self.cap = cv2.VideoCapture(0)
    self.iid = 0
    
    self.create_out_dir()
    self.create_csv_file()
    self.record_save_images()

  def create_out_dir(self):
    if exists(self.output_dir):
      cont = input(f'{self.output_dir} directory already exists. Continue and append? y/n: ')
      if cont.strip().lower() != 'y':
        exit()
    else:
      print(f'Creating directory {self.output_dir}...', end=' ')
      os.makedirs(self.output_dir)
      print('Done.')

  def create_csv_file(self):
    if exists(f'{self.output_dir}.csv'):
      cont = input(f'{self.output_dir}.csv already exists. Continue and append? y/n: ')
      if cont.strip().lower() != 'y':
        exit()
      self.csv_file =  open(f'{output_dir}.csv', 'a') 
      self.df = pd.read_csv(f'{output_dir}.csv', sep=';')
      self.gesture_counts = self.df.gesture.value_counts().to_dict() 
    else:
      print(f'Creating file {self.output_dir}.csv...', end=' ')
      self.csv_file =  open(f'{output_dir}.csv', 'a') 
      self.csv_file.write('gesture;image\n')
      self.gesture_counts = {}
      print('Done.')

  def record_save_images(self):
    while self.cap.isOpened():
      try:
        _, frame = self.cap.read()
        cv2.imshow('Recording', frame)

        key = cv2.waitKey(1)
        if key != -1: 
          key = chr(key)
          if key in self.keys_dict:
            self.capture_gesture(self.keys_dict[key])
            self.iid += 1
            self.add_print_gesture_count(self.keys_dict[key])
            
          elif key == 'q': # quit program
            print('Quitting...')
            self.shutdown()

      except(KeyboardInterrupt):
        self.shutdown()

  def capture_gesture(self, gesture_name):
      print(f'Capturing {gesture_name} gesture')
      file_name = create_filename()
      file_name += f'_{gesture_name + str(self.iid)}'
      _, frame = self.cap.read()
      cv2.imshow(gesture_name, frame)
      key = cv2.waitKey(1)
      file_name += '.png'
      cv2.imwrite(join(self.output_dir, file_name),  frame)
      self.csv_file.write(f'{gesture_name};{file_name}\n')

  def add_print_gesture_count(self, gesture_name):
    if gesture_name in self.gesture_counts:
      self.gesture_counts[gesture_name] += 1
    else:
      self.gesture_counts[gesture_name] = 1

    print(self.gesture_counts)

  def shutdown(self):
    self.csv_file.close()
    self.cap.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
  if len(sys.argv) > 2:
    print_usage()
    exit()

  output_dir = 'data' if len(sys.argv) == 1 else sys.argv[1]
  print(f'Output directory: {output_dir}/')

  data_collector = Data_Collector(output_dir)


