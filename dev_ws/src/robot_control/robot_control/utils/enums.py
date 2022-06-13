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
