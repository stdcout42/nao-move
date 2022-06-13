import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nao_move_interfaces.msg import BotState 
from nao_move_interfaces.msg import GuiCmd

os.environ['KIVY_NO_ARGS'] = '1' 
import kivy
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.dropdown import DropDown
from kivy.core.window import Window
from kivy.uix.textinput import TextInput
from kivy.clock import Clock
from enum import Enum, auto
from robot_control.sim_subscriber import AutoName

class Command(AutoName):
  DRAW_CIRCLE = auto()

class GuiPublisher(Node):
  def __init__(self):
    super().__init__('gui_publisher')
    self.command_publisher = self.create_publisher(GuiCmd, 'gui', 10)
    self.bot_state_subscription = self.create_subscription(BotState, 'bot_state', 
        self.bot_state_cb, 10)
    self.bot_state = BotState()

  def bot_state_cb(self, msg):
    #self.get_logger().info(f'Incoming bot state {msg}')
    self.bot_state = msg

  def publish_command(self, cmd, shape, mod, obj, args):
    gui_cmd = GuiCmd()
    gui_cmd.cmd = cmd
    gui_cmd.shape = shape
    gui_cmd.shape_mod = mod
    gui_cmd.args = args
    gui_cmd.obj = obj
    self.command_publisher.publish(gui_cmd)

class Gui(App):
  HEIGHT = 360 
  WIDTH = 480 
  BUTTON_OFF_Y = 15

  mode = 'IMITATE'
  subject_name = 'subject_name'
  test_shape = 'test_shape'
  elapsed_time = 0
  timer_is_on = False
  bot_state = None

  cmd_btn_txts = ['stop', 'draw', 'feedback', 'record',
      'replay', 'move', 'clean', 'spawn', 'name']
  cmd_dropdown_btns = None

  shape_btn_txts = ['circle', 'triangle', 'square']
  shape_dropdown_btns = None

  mod_btn_txts = ['no_mod', 'left', 'left', 'right', 'up', 'down', 'pitch',
      'roll', 'yaw', 'bigger', 'smaller', 'slower', 'faster', 'draw']
  mod_dropdown_btns = None

  obj_btn_txts = ['table', 'tray', 'ball']
  obj_dropdown_btns = None


  def __init__(self):
    super(Gui, self).__init__()
    self.init_ros_node()
    self.cmd_dropdown = self.get_dropdown(self.cmd_btn_txts)
    self.shapes_dropdown = self.get_dropdown(self.shape_btn_txts)
    self.mods_dropdown = self.get_dropdown(self.mod_btn_txts)
    self.objs_dropdown = self.get_dropdown(self.obj_btn_txts)

    self.mode_label = Label(text=self.get_mode_label_txt(),
        pos=(80, self.HEIGHT-40), size_hint=(0.3, 0.1))
    
    self.subject_name_label = Label(text=self.subject_name,
        pos=(200, self.HEIGHT-40), size_hint=(0.3, 0.1))

    self.test_shape_label = Label(text=self.test_shape,
        pos=(300, self.HEIGHT-40), size_hint=(0.3, 0.1))

    self.command_label = Label(text='Command',
        pos=(10, self.HEIGHT-80), size_hint=(0.15, 0.1))

    self.cmd_dropdown_btn = Button(text='Cmd',
       pos=(10, self.HEIGHT-110), size_hint=(0.15,0.1), background_color=(0.1,0,1,1))
    self.cmd_dropdown_btn.bind(on_release=self.cmd_dropdown.open)
    self.cmd_dropdown.bind(on_select=lambda instance, x: setattr(self.cmd_dropdown_btn, 'text', x))

    self.shapes_dropdown_btn = Button(text='Shape',
       pos=(10, self.HEIGHT-150), size_hint=(0.15,0.1))
    self.shapes_dropdown_btn.bind(on_release=self.shapes_dropdown.open)
    self.shapes_dropdown.bind(on_select=lambda instance, x: setattr(self.shapes_dropdown_btn, 'text', x))

    self.mods_dropdown_btn = Button(text='Mod',
       pos=(90, self.HEIGHT-150), size_hint=(0.15,0.1))
    self.mods_dropdown_btn.bind(on_release=self.mods_dropdown.open)
    self.mods_dropdown.bind(on_select=lambda instance, x: setattr(self.mods_dropdown_btn, 'text', x))


    self.objs_dropdown_btn = Button(text='Object',
       pos=(10, self.HEIGHT-190), size_hint=(0.15,0.1))
    self.objs_dropdown_btn.bind(on_release=self.objs_dropdown.open)
    self.objs_dropdown.bind(on_select=lambda instance, x: setattr(self.objs_dropdown_btn, 'text', x))

    self.args_input = TextInput(text='arg0 arg1 arg2', multiline=False,
        pos=(10, self.HEIGHT-230), size_hint=(0.35, 0.1))

    self.send_command_button = Button(text='Send cmd', 
        pos=(10, self.HEIGHT-270), 
        size_hint=(0.15,0.1), background_color=(0,1,0,1))
    self.send_command_button.bind(on_press=self.send_cmd_pressed)


    ############### 2nd column below

    self.timer_button = Button(text='Start/Stop timer', 
        pos=(270, self.HEIGHT-110), size_hint=(0.25,0.1), background_color=(0,1,0,1))
    self.timer_button.bind(on_press=self.start_timer_pressed)

    self.timer_label = Label(text=self.get_timer_elapsed_time_str(),
      pos=(370, self.HEIGHT-110), size_hint=(0.15, 0.1))

    self.reset_timer_button = Button(text='Reset timer', 
        pos=(270, self.HEIGHT-150), size_hint=(0.25,0.1))
    self.reset_timer_button.bind(on_press=self.reset_timer_pressed)

    self.save_to_file_button = Button(text='Save time to file', 
        pos=(250, self.HEIGHT-190), size_hint=(0.35,0.1), 
        background_color=(0.7,0.5,0.1,1))
    self.save_to_file_button.bind(on_press=self.save_button_pressed)


    self.set_schedule_intervals()

  def build(self):

    Window.size = (self.WIDTH, self.HEIGHT)
    layout = FloatLayout()

    layout.add_widget(self.mode_label)
    layout.add_widget(self.subject_name_label)
    layout.add_widget(self.test_shape_label)
    layout.add_widget(self.command_label)
    layout.add_widget(self.send_command_button)
    layout.add_widget(self.args_input)
    layout.add_widget(self.timer_button)
    layout.add_widget(self.timer_label)
    layout.add_widget(self.reset_timer_button)
    layout.add_widget(self.save_to_file_button)
    layout.add_widget(self.cmd_dropdown_btn)
    layout.add_widget(self.shapes_dropdown_btn)
    layout.add_widget(self.mods_dropdown_btn)
    layout.add_widget(self.objs_dropdown_btn)


    return layout 

  def get_dropdown(self, btn_txts):
    dropdown = DropDown()
    for txt in btn_txts:
      btn = Button(text=txt, size_hint_y=None, height=38)
      btn.bind(on_release= lambda btn: dropdown.select(btn.text))
      dropdown.add_widget(btn)
    return dropdown

  def get_mode_label_txt(self):
     return f'Mode: {self.mode}'

  def get_timer_elapsed_time_str(self):
    return f'{self.elapsed_time}s'

  def init_ros_node(self):
    rclpy.init(args=None)
    self.gui_publisher  = GuiPublisher()

  def set_schedule_intervals(self): 
    self.node_schedule = Clock.schedule_interval(self.spin_node, 1)
    self.bot_stat_schedule = Clock.schedule_interval(self.check_bot_state, 1)

  def start_timer_pressed(self, instance):
    if self.timer_is_on:
      self.timer_is_on = False
      self.timer_button.background_color = (0,1,0,1)
    else:
      self.timer_is_on = True
      self.timer_button.background_color = (1,0,0,1)
      self.timer_schedule = Clock.schedule_interval(self.increase_time, 1)

  def save_button_pressed(self, instance):
    if self.subject_name:
      f =  open(f'src/robot_control/robot_control/experiments/{self.subject_name}', 'a')
      f.write(f'{self.test_shape};{self.elapsed_time}\n')
      f.close()


  def reset_timer_pressed(self, instance):
    self.elapsed_time = 0
    self.timer_label.text = self.get_timer_elapsed_time_str()

  def increase_time(self, dt):
    if self.timer_is_on:
      self.elapsed_time += 1
      self.timer_label.text = self.get_timer_elapsed_time_str()
    return self.timer_is_on

  def spin_node(self, dt):
    rclpy.spin_once(self.gui_publisher, timeout_sec=0.001)
 
  def check_bot_state(self, dt):
    bot_state = self.gui_publisher.bot_state
    if bot_state:
      if bot_state.mode_changed and \
        bot_state.mode_name != self.mode:
          self.mode = bot_state.mode_name
          self.mode_label.text = self.get_mode_label_txt()
      if bot_state.subject_name:
        if bot_state.subject_name != self.subject_name:
          self.subject_name = bot_state.subject_name
          self.subject_name_label.text = self.subject_name 
        
      if bot_state.test_shape:
        self.test_shape = bot_state.test_shape
        self.test_shape_label.text = self.test_shape

      self.bot_state = bot_state

  def send_cmd_pressed(self, instance):
    cmd = self.cmd_dropdown_btn.text  
    shape = self.shapes_dropdown_btn.text
    mods = self.mods_dropdown_btn.text
    args = self.args_input.text
    obj = self.objs_dropdown_btn.text

    cmd = cmd if cmd != 'Cmd' else ''
    shape = shape if shape != 'Shape' else ''
    mods = mods if mods != 'Mod' else ''
    obj = obj if obj != 'Object' else ''
    args = args if args != 'arg0 arg1 arg2' else ''
    
    self.gui_publisher.publish_command(
        cmd,
        shape,
        mods,
        obj,
        args)


def main(args=None):
  gui = Gui()
  gui.run()

if __name__ == '__main__':
  main()
