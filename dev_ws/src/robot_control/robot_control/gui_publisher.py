import os
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nao_move_interfaces.msg import BotState 

os.environ['KIVY_NO_ARGS'] = '1' 
import kivy
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
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
    self.command_publisher = self.create_publisher(String, 'gui', 10)
    self.bot_state_subscription = self.create_subscription(BotState, 'bot_state', 
        self.bot_state_cb, 10)
    self.bot_state = BotState()

  def bot_state_cb(self, msg):
    #self.get_logger().info(f'Incoming bot state {msg}')
    self.bot_state = msg

  def publish_command(self, command):
    msg = String()
    msg.data=command
    self.command_publisher.publish(msg)
    

class Gui(App):
  HEIGHT = 360 
  WIDTH = 480 
  BUTTON_OFF_Y = 15
  mode = 'IMITATE'

  def __init__(self):
    super(Gui, self).__init__()
    self.init_ros_node()

    self.mode_label = Label(text=self.get_mode_label_txt(),
        pos=(150, self.HEIGHT-30), size_hint=(0.3, 0.1))

    self.command_label = Label(text='Command',
        pos=(10, self.HEIGHT-40), size_hint=(0.2, 0.1))

    self.send_command_button = Button(text='Send cmd', 
        pos=(120, self.HEIGHT-70), 
        size_hint=(0.2,0.1))

    self.send_command_button.bind(on_press=self.send_command_cb)

    self.command_input = TextInput(text='', multiline=False,
        pos=(10, self.HEIGHT-70), size_hint=(0.2, 0.1))
 
    self.set_schedule_intervals()

  def build(self):

    Window.size = (self.WIDTH, self.HEIGHT)
    layout = FloatLayout()

    layout.add_widget(self.mode_label)
    layout.add_widget(self.command_label)
    layout.add_widget(self.send_command_button)
    layout.add_widget(self.command_input)

    return layout 

  def get_mode_label_txt(self):
     return f'Mode: {self.mode}'

  def init_ros_node(self):
    rclpy.init(args=None)
    self.gui_publisher  = GuiPublisher()

  def set_schedule_intervals(self): 
    self.node_schedule = Clock.schedule_interval(self.spin_node, 1)
    self.bot_stat_schedule = Clock.schedule_interval(self.check_bot_state, 1)

  def spin_node(self, dt):
    rclpy.spin_once(self.gui_publisher, timeout_sec=0.001)
 
  def check_bot_state(self, dt):
    bot_state = self.gui_publisher.bot_state
    if bot_state and bot_state.mode_changed and \
        bot_state.mode_name != self.mode:
          self.mode = bot_state.mode_name
          self.mode_label.text = self.get_mode_label_txt()

  def send_command_cb(self, instance):
    self.gui_publisher.publish_command(self.command_input.text)


def main(args=None):
  gui = Gui()
  gui.run()

if __name__ == '__main__':
  main()
