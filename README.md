

# PRESS: Programming Robots Effectively through Speech and Sign language
End-user programming of robot trajectories by using natural communication
## Description
This is a framework developed as part of Saman Shahbazi's bachelor thesis at The Free University Amsterdam. 

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
- If you have a mic other than your computer's default microphone,you have to set `MIC_INPUT` in `dev_ws/src/robot_control/robot_control/SpeechController.py` and set `EXT_MIC = True` 

### Run
If the installation of all dependencies and ROS2 went well, you have to `cd` to the directory `/nao-move/dev_ws` and
run

First make sure you've sourced the ROS2 environment, then:
- `colcon build`
- `. install/setup.bash` (assuming you're in bash)
- `ros2 launch robot_control robot_control.launch.py`.

To run with the the GUI to set up for testing / experiment, for the last step run:
`ros2 launch robot_control robot_control_experiment.launch.py` instead.

#### How to interact
![Robot modes](docs/images/robot_modes.png)

The sign language commands can be found [here](docs/flipped_demos).
  
Once in feedback mode, the following commands can be used to adjust the saved trajectory:
![feedback commands](docs/images/feedback_cmds.png)


## ROS architecture
Topics are an essential component of the ROS network as they provide a means for nodes to exchange messages with one another. Seven ROS2 topics and three ROS messages were developed for this project to act as communication routes between the nodes.

The most important ROS node, SimController, is subscribed to all topics except bot state and demo. The actions taken by the SimController upon receiving these messages are described in Section 3.2.4.1.
### wrist_coords topic
The wrist_coords topic uses a message named WristCoordinates. The message is a container for three Vector3 ROS message types (each Vector3 contains three floats, for x, y, and z dimensions), which correspond to the right-wrist, left-wrist MediaPipe Pose world coordinates (coordinates relative to the hip), and a right-wrist coordinate normalized to [0.0, 1.0] by the image width and height, respectively.

The visual component (CamController in Figure 2) publishes the coordinates of the wrist generated by MediaPipe's pose estimator in the wrist_coords topic. The SimController, which is subscribed to the wrist_coords topic, is able to receive these coordinates and act on them.
### sign_lang topic
This topic uses a standard ROS message (string) as its message type. The CamController module classifies sequences of 30 frames into sign language and publishes them to the sign_lang topic. The SimController, which is subscribed to this topic, is then able to receive the sign language and respond accordingly.
### speech topic
The SpeechController uses a standard ROS message type (string) to publish publish keywords that were heard via the microphone to the speech topic.
### movement topic
When the robot is in move mode, the movement topic relays direction information interpreted by the CamController using a standard message type (string).
### gui topic
The gui topic uses the GuiCmd message, which contains six string fields: cmd, shape, shape_mod, obj, args, which are used by the GuiController to control the simulation environment. The cmd field can be one of sixteen commands [full list in the Appendix section z], whereas the other fields supplement this main command. As an example, cmd: spawn, obj: table is used to spawn a table in the simulation.
### demo topic
The demo topic uses the standard ROS message type (string). The GuiController uses this topic to change the sign language demo videos shown on screen to assist users when they're interacting with the framework. This feature was created for user research, but it can also be used for other purposes, such as familiarizing a user with the framework's sign languages.
### bot_state topic
The bot_state topic uses the BotState message [see Appendix section z1] developed to notify subscribed nodes of changes to the robot's state (such as the current mode). The node SimController publishes to this topic, and the CamController and GuiController are the nodes that have a subscription to it. 

The CamController uses the received data to determine what type of information to publish (coordinates or directions) and to display informative messages to the user. Whereas the GuiController receives the RMSE of the experiment findings, as well as other robot state information, and saves them to disk for subsequent analysis.
