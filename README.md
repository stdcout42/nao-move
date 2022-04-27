

# nao-move: implementing multi-modal control on Nao/Pepper robots using ROS
## docs
- [raw logbook](https://github.com/stdcout42/nao-move/blob/main/docs/raw_logbook.md)
- [summarized logbook](https://github.com/stdcout42/nao-move/blob/main/docs/logbook.md)

## Google Drive
[Link](https://drive.google.com/drive/folders/15IWhDwY0hzxDpBRG4-ujRWeK-qrg7z2i?usp=sharing)
- results / data folder
    - Robot arm simulator demo [video](https://drive.google.com/file/d/1QnhcysKL1fhK-SB1o6wS4MHmiphN7qrs/view)

## Installation
### Requirements / dependencies
- [ros2-foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- tensorflow (`pip3 install tensorflow`)
- numpy (`pip3 install numpy`)
- pyaudio  (`sudo apt install portaudio19-dev python3-pyaudio` `pip3 install pyaudio` on Ubuntu 20.04)
- vosk (`pip3 install vosk`)
- TODO: more?
### Configuration
- Currently hardcoded to detect webcam from feed 0 (on linux: `/dev/video0`)
### Run
From the directory `/nao-move/dev_ws` run `ros2 launch arm_simulator arm_simulator.launch.py`.

## hand-gesture-recognition-using-mediapipe
[Link](https://github.com/kinivi/hand-gesture-recognition-mediapipe) - which is a translation of a [repo](https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe) which is in Japanese. 
- Keypoints (landmarks) of hand(s) are detected using media-pipe
- Library includes helper files to record data-set
- Keypoints are fed to a (regressino) neural network to classify hand gestures

The library is slightly modified to be able to record more class captures as well as to able to detect more gestures (thus the neural network's output layer is modified).

Currently, the following six gestures are classified:
- fist
- open hand
- up
- down
- left 
- right

![Demo of gestures](gestures_demo.gif "Demo of gestures")

## ROS2 architecture
### Nodes
A node for each purpose.
#### Simulation listener (`sim_listener.py`)
- Node that listens to gesture + audio + coordinates and moves the arms accordingly
- TODO: When the node hears the 'record' keyword, it will record the movements starting the first fist gesture capture
and ending when it sees the stop (open hand) gesture or when the keyword 'stop' is heard.
- TODO: When the there exists a saved trajectory, the trajectory can be played back when the 'repeat' keyword is heard.
- TODO: A saved trajectory can be adjusted through feedback with the keywords 'big', 'small', 'left', 'right', 'up', 'down'
#### Gesture (and coordinates) talker (`gesture_talker.py`)
- Node that transmits gestures as well as coordinates (since coordinates are derived from the gesture coordinates) to Arm Control
#### Speech talker (`speech_talker.py`)
- Node that transmits keywords recognized by the speech recognizer to Arm Control
