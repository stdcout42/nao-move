import rclpy
from rclpy.node import Node
import pyaudio
import json
import threading

from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
from vosk import SetLogLevel 


class SpeechPublisher(Node):
  # MIC_INPUT is set to the external mic input, the value of which 
  # has been retrieved by the pyaudio_test.py script.
  # On Ubuntu 20.04 this index is of the input source 'pulse'
  MIC_INPUT = 10

  def __init__(self):
    super().__init__('speech_talker')
    self.publisher_ = self.create_publisher(String, 'speech', 10)
    self.msg = String()
    SetLogLevel(-1)

    threading.Thread(target=self.classify_stream).start()
    #self.classify_stream()

  def classify_stream(self):
    self.get_logger().info('speech in the house')
    model = Model('src/robot_control/robot_control/vosk_model')
    recognizer = KaldiRecognizer(model, 16000)
    pa = pyaudio.PyAudio()
    stream = pa.open(format=pyaudio.paInt16, rate=16000, channels=1, input=True, frames_per_buffer=8192, input_device_index=self.MIC_INPUT)
    stream.start_stream()

    while rclpy.ok():
      try:
        data = stream.read(4096)
        if recognizer.AcceptWaveform(data):
          res = recognizer.Result()
          #print(res)
          js = json.loads(res)
          first_two_words = js['text'].split(' ')[:2]
          if len(first_two_words) == 2 and first_two_words[0] == 'hey':
            #self.get_logger().info(f'first_two_words {first_two_words}')
            msg = String()
            msg.data = first_two_words[1]
            self.publisher_.publish(msg)

      except KeyboardInterrupt:
        exit()

    
def main(args=None):
  rclpy.init(args=args)
  speech_publisher = SpeechPublisher()

  try:
    rclpy.spin(speech_publisher)
  except KeyboardInterrupt:
    speech_publisher.destroy_node()
    rclpy.shutdown()
  finally:
    speech_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
