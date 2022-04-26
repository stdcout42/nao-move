import rclpy
from rclpy.node import Node
import pyaudio
import json

from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

class SpeechPublisher(Node):
  def classify_stream(self):
    model = Model('src/arm_simulator/arm_simulator/vosk_model')
    recognizer = KaldiRecognizer(model, 16000)
    pa = pyaudio.PyAudio()
    stream = pa.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8192)
    stream.start_stream()

    while rclpy.ok():
      data = stream.read(4096)
      if recognizer.AcceptWaveform(data):
        res = recognizer.Result()
        js = json.loads(res)
        first_word = js['text'].split(' ')[0]
        if first_word != "":
          print(first_word)
          msg = String()
          msg.data = first_word
          self.publisher_.publish(msg)


  def __init__(self):
    super().__init__('speech_talker')
    self.publisher_ = self.create_publisher(String, 'speech', 10)
    self.msg = String()
    self.classify_stream()
    
def main(args=None):
  rclpy.init(args=args)

  speech_publisher = SpeechPublisher()

  rclpy.spin(speech_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  speech_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
