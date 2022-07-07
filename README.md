

# nao-move: End-user programming of robot trajectories by using natural communication
## Description
This is a framework developed for a bachelor thesis at The Vrije University at Amsterdam. 

The goal of this project is to enable non-expert users to use natural communication (speech and sign language) to interact with and create robot trajectories (in simulation). 

This project draws inspiration from several sources: 
1. This work is built on preliminary work done by a team of Bachelor students at UT Austin in the Socially Intelligent Machines Lab, who created a framework that is able to generate robot trajectories based on demonstrations gathered from a visual input system (a single lens camera).

From (2) and (3) code snippets were taken and modified to fit for respectively, static and dynamic hand sign classifications:
2. Static [Hand gesture recognition](https://github.com/kinivi/hand-gesture-recognition-mediapipe) library - which is a translation of a [repo](https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe) which is in Japanese. To classify and train (static) hand gestures.
3. Dynamic [hand sign language recognition](https://github.com/nicknochnack/ActionDetectionforSignLanguage), to classify sign language and draw a probablity visualization.


## Installation
### Requirements / dependencies
- python3
- [ros2-foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [colcon](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#install-colcon) (`sudo apt install python3-colcon-common-extensions`)
- tensorflow (`pip3 install tensorflow`)
- mediapipe (`pip3 install mediapipe`)
- pybullet (`pip3 install pybullet`)
- qibullet (`pip3 install qibullet`)
- opencv (`pip3 install opencv-python`)
- numpy (`pip3 install numpy`)
- pyaudio  (`sudo apt install portaudio19-dev python3-pyaudio` `pip3 install pyaudio` on Ubuntu 20.04)
- vosk (`pip3 install vosk`)
- pynput (`pip3 install pynput`)
- kivy  (`pip3 install kivy`)
- playsound (`pip3 install playsound`)
- xclip (`sudo apt install xclip`)
- [gTTs](https://gtts.readthedocs.io/en/latest/) (`pip3 install gTTs`)


### Configuration
- Currently detects webcam from feed 0 (on linux: `/dev/video0`), if you have multiple video inputs, this might need to be changed in (change `VIDEO_SRC` in `dev_ws/src/robot_control/robot_control/utils/cvutils.py`)
- Set `MIC_INPUT` in `dev_ws/src/robot_control/robot_control/SpeechController.py` 

### Run
If the installation of all dependencies and ROS2 went well, you have to `cd` to the directory `/nao-move/dev_ws` and
run

First make sure you've sourced the ROS2 environment, then:
- `colcon build`
- `. install/setup.bash` (assuming you're in bash)
- `ros2 launch robot_control robot_control.launch.py`.

#### How to interact
![Robot modes](docs/images/robot_modes.png)

The sign language commands can be found [here](docs/flipped_demos).
  
Once in feedback mode, the following commands can be used to adjust the saved trajectory:
![feedback commands](docs/images/feedback_cmds.png)



