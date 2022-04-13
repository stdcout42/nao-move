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

def main():
  if len(sys.argv) > 2:
    print_usage()
    exit()

  output_dir = 'data' if len(sys.argv) == 1 else sys.argv[1]
  print(f'Output directory: {output_dir}/')

  create_out_folder(output_dir)
  record_save_images(output_dir)


def print_usage():
  print(f'Usage:\n\t python3 collect_data.py [output_folder]')


def create_out_folder(output_dir):
  if exists(output_dir):
    cont = input(f'{output_dir} already exists. Continue and append? y/n: ')
    if cont.strip().lower() != 'y':
      exit()
  else:
    print(f'Creating directory {output_dir}...', end=' ')
    os.makedirs(output_dir)
    print('Done.')


def create_filename():
  dt = datetime.datetime.now()
  year, month, day, hour, minute, second  = dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second
  return "{}{}{}_{}_{}_{}".format(year, month, day, hour, minute, second)


def shutdown(csv_file, cap, cv2):
  csv_file.close()
  cap.release()
  cv2.destroyAllWindows()

# Function is doing too much atm, needs to be modularized better
def capture_gesture(cap, iid, cv2, gesture_name, csv_file, output_dir):
    print(f'Capturing {gesture_name} gesture')
    file_name = create_filename()
    file_name += f'_{gesture_name + str(iid)}'
    _, frame = cap.read()
    cv2.imshow(gesture_name, frame)
    key = cv2.waitKey(1)
    file_name += '.png'
    cv2.imwrite(join(output_dir, file_name),  frame)
    csv_file.write(f'{gesture_name};{file_name}\n')

"""
Waits for key input and captures and saves TODO: elaborate
"""
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
def record_save_images(output_dir):
  cap = cv2.VideoCapture(0)
  iid = 0
  with open(f'{output_dir}.csv', 'a') as csv_file:
    while cap.isOpened():
      try:
        _, frame = cap.read()
        cv2.imshow('Recording', frame)

        key = cv2.waitKey(1)
        if key != -1: 
          key = chr(key)
          if key in keys_dict:
            capture_gesture(cap, iid, cv2, keys_dict[key], csv_file, output_dir)
            iid += 1

          elif key == 'q': # quit program
            print('Quitting...')
            shutdown(csv_file, cap, cv2)

      except(KeyboardInterrupt):
        shutdown(csv_file, cap, cv2)
        

if __name__ == "__main__":
  main()
