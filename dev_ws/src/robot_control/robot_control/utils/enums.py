from enum import Enum, auto

class AutoName(Enum):
  def _generate_next_value_(name, start, count, last_values): #pylint: disable=no-self-argument
    return name

class Obj(AutoName):
  BALL = auto()
  TRAY = auto()
  TABLE = auto()

class SignMode(AutoName):
  WAITING_FOR_HEY = auto()
  HEY_RECEIVED = auto()
  OFF = auto()

class VideoDemo(Enum):
  HEY = 'hey.mkv'
  PLAY = 'play.mkv'
  RECORD = 'record.mkv'

def get_video_demo_from_str(vd_str):
  for vid_dem in list(VideoDemo):
    if vid_dem.name == vd_str:
      return vid_dem
  return None

class Shape(AutoName):
  CIRCLE = auto()
  SQUARE = auto()
  TRIANGLE = auto()
  HORIZONTAL_LINE = auto()
  VERTICAL_LINE = auto()
  DIAGONAL_LINE = auto()
