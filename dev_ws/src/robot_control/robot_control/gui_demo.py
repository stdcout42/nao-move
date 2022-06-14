import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from os.path import join

os.environ['KIVY_NO_ARGS'] = '1' 
import kivy
from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.image import Image
from kivy.uix.video import Video
from kivy.uix.videoplayer import VideoPlayer
from kivy.uix.floatlayout import FloatLayout
from kivy.core.window import Window
from kivy.clock import Clock
from .utils.enums import VideoDemo, get_video_demo_from_str

RED_COLOR = '[color=ff3333]'
COLOR_CLOSE = '[/color]'


class GuiSubscriber(Node):
  def __init__(self):
    super().__init__('gui_listener')
    self.demo_subscription = self.create_subscription(String, 'demo', self.demo_cb, 10)
    self.video_demo = VideoDemo.HEY

  def demo_cb(self, msg):
    self.get_logger().info(f'Incoming bot state {msg}')
    self.video_demo = get_video_demo_from_str(msg.data)
  
class GuiDemo(App):
  HEIGHT = 720 
  WIDTH = 640 
  mode = 'IMITATE'
  subject_name = 'subject_name'
  curr_video_demo = VideoDemo.HEY
  VIDEO_DEMO_PATH = 'src/robot_control/robot_control/demo_vids/'
  
  def __init__(self):
    super(GuiDemo, self).__init__()
    self.init_ros_node()

    self.demo_label = Label(text=self.get_demo_label_text(),
        pos=(260, self.HEIGHT-80), size_hint=(0.1, 0.1), markup=True, font_size=50)

    self.video_demo_player = VideoPlayer(source= join(self.VIDEO_DEMO_PATH, self.curr_video_demo.value),
        state='play', options={'eos': 'loop'})
    self.set_schedule_intervals()

  def build(self):
    Window.size = (self.WIDTH, self.HEIGHT)
    layout = FloatLayout()

    layout.add_widget(self.demo_label)
    layout.add_widget(self.video_demo_player)
    return layout 

  def init_ros_node(self):
    rclpy.init(args=None)
    self.gui_subscriber  = GuiSubscriber()

  def set_schedule_intervals(self): 
    self.node_schedule = Clock.schedule_interval(self.spin_node, 1)
    self.bot_stat_schedule = Clock.schedule_interval(self.check_listener, 1)

  def get_demo_label_text(self):
    return f'{RED_COLOR}{self.curr_video_demo.name}{COLOR_CLOSE}'

  def spin_node(self, dt):
    rclpy.spin_once(self.gui_subscriber, timeout_sec=0.01)
 
  def check_listener(self, dt):
    video_demo = self.gui_subscriber.video_demo 
    if video_demo != self.curr_video_demo:
      self.curr_video_demo = video_demo
      self.video_demo_player.source = join(self.VIDEO_DEMO_PATH, video_demo.value)
      self.demo_label.text = self.get_demo_label_text()

def main(args=None):
  gui = GuiDemo()
  gui.run()

if __name__ == '__main__':
  main()
