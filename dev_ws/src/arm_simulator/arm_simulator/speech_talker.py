import rclpy
from rclpy.node import Node
import pyaudio
import json

from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

class SpeechPublisher(Node):
  # MIC_INPUT is set to the external mic input, the value of which 
  # has been retrieved by the pyaudio_test.py script.
  MIC_INPUT = 5

  def classify_stream(self):
    model = Model('src/arm_simulator/arm_simulator/vosk_model')
    recognizer = KaldiRecognizer(model, 16000)
    pa = pyaudio.PyAudio()
    stream = pa.open(format=pyaudio.paInt16, rate=16000, channels=1, input=True, frames_per_buffer=8192, input_device_index=self.MIC_INPUT)
    stream.start_stream()

    while rclpy.ok():
      try:
        data = stream.read(4096)
        if recognizer.AcceptWaveform(data):
          res = recognizer.Result()
          js = json.loads(res)
          first_two_words = js['text'].split(' ')[:2]
          if len(first_two_words) == 2 and first_two_words[0] == 'hey':
            print(f'first_two_words {first_two_words}')
            msg = String()
            msg.data = first_two_words[1]
            self.publisher_.publish(msg)

      except KeyboardInterrupt:
        exit()


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
