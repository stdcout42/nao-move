# Implementing control on Nao/Pepper robots using ROS

## Meeting March 21st

1. Teachable machines (play around)
2. Run the code from the group locally (and record it)
3. Qibullet?
4. Later port all of this to the Nao
5. create drive for all shared folders

---

## Monday April 4th

Ran:

```
- simulation/src/example.py
- simulation/src/keyboard_mover.py

```

### sim_listener.py and ball_track_publisher.py

1. Indent error on line 70
2. absolute path to `jaco_robotiq_object.urdf` fixed.
3. `[ERROR] [1649072219.742673]: Unable to immediately register with master node [http://localhost:11311]: master may not be running yet. Will keep trying.`
4. Fixed by running `roscore`
5. 4 threads too little - going to max (8)
6. Able to run it now. Need to figure out what the chatter communication consists of.
7. No need for 6. Ran `simlistener.py` and ran `sim_publisher.py` -- needed to enable webcam on virtmanager 

First impression: Pretty cool and lots of new stuff to learn. The code is quick, and when it works, it works well, however, the recognition is not really that well for the ball tracker. It seems that it’s just looking for _anything_ that is green. Moreover, the ball is not detected in  the lower right corner (my right) of the screen. A ML library will probably improve things (but also add overhead).

## Tuesday April 5th

1. Looked through the hand movement code - it’s not quite complete as far I can tell. However using mediapipe, and a a custom dataset, hand gesture recognition should be trivial. 

2. Will spend some more time looking at and understanding the `sim_listener.py`, and publisher code. 

3. Installed qibullet, installed ROS wrapper (interface), start reading on ROS/QiBullet combo

4. Need a qibullet wrapper (to be able to use naoqi instance)?
  
 ## Wednesday April 6th
 1. A little bit confused - installed NaoQi wrappper however, I need a robot connection... I'm just trying to move the nao i QiBullet... OK so NaoQi API is for making the actual robot move D'oh! ... Should I stop trying to make the simulation work and just work in the lab with the real thing / NaoQi? I think that's cleared up now.
 2. Trying to get QiBullet simulation to walk. However, it seems that only low level motions are implemented. Should I try to get the simulation working for motions or should I learn more about NaoQi? What about Webot? Will look at that for a bit.
 3. While webot works - should I be spending time learning this framework? Can I connect naoqi to webots? Repository naoqism which attemmpts to do just that is deprecated.

## Meeting April 7th
-  test out, what do we need to add (in terms of fine tuning)
-  run code: make work with gestures/speech (for now on robot arm)

In parallel, think of use cases:
- using tools for ex
- literature on multi-modal
For these tasks think of goals the tasks need to accomplish.
Also, think of baselines to improve on.

Article to read:
P. M. Pilarski, M. R. Dawson, T. Degris, F. Fahimi, J. P. Carey, and R. S. Sutton, ‘‘Online human training of a myoelectric prosthesis controller via actor-critic reinforcement learning,’’ in Proc. IEEE Int. Conf. Rehabil. Robot., Jun. 2011, pp. 1–7.

Literature to look up: multi-modal interfacing
Authors: Henny Admoni, Maya Cakmak

## April 7th
1. In the README.md it is stated that feedback via gestures is _not_ implemented.
2. Feedback via voice is supposedly implemented - however, I'm not quite catching on how and where that is done. Getting some errors trying to start up those nodes. Tomorrow I will attempt to start up use teachable-machines to set up my own voice commands, as well as understand pybullet movement better.

## April 8th
1. created a small voice-recognition [here](https://teachablemachine.withgoogle.com/models/YvewNdQBL/) 
2. After some dependency circulation problems, and switching between distributions, was able to save it to a kerasmodel to load in python 
3. It's becoming a little bit hard to actually find my way around the teachable model as it only provides the model and no instructions how to actually work with this model. I will now attempt to create my own model (tensorflow).
4. Seemed overkill and overly complicated. Seems like the library speech\_recognition might is the easier way out.

## April 11th
1. Trying pocketsphinx for word recognition
2. New API available: vosk
3. Got the new voice commands working with the feedback topic

## April 12th
1. Focus will be on getting gestures working. Available are a few models for predicting hand gestures. The feedback to the robot arm is not implemented. The models are in notebook form. I will examine these first. The capturing image script will come in handy.

## April 13th
1. The capturing script was not behaving as I was expecting. Wrote a new one. Need to acquire new data. This will probably take some time but I will a subset of the classes.
2. Will also create a new classifier for gestures.
3. Collected (webcam) pictures for up, down, left, right (for now).

## April 14-18th
Been busy with setting up a new computer, celebrating Easter but also been working on the hand gesture classifier. 
1. Wrote a new image collecting script
2. Tested the images on classifiers derived through transferlearning (EfficientNet). The preliminary results were underwhelming. On a 4 categorical classifier (4 gestures), with images that have various backgrounds the classifier performs <= 70% accuracy. Most likely this has to do with the images being non cropped (but perhaps also of llow quality due to my webcam). Seems like there is no way around and MediaPipe has to be involved in the pipeline for the landmark detection, then feed this to another classifier which classifies the gestures. Will work on this today. Also have a new webcam coming in tonight.

## April 19th
1. Luckily, I found a repository that does exactly what is required (mediapipe hand recognition + gesture classification), this saves me a lot of time and work. 
2. The preliminary results are very promising. Next will be feeding the gestures to the pybullet arm.
3. Also, I want to convert everything to ROS2... Will have to see what Dr. Baraka thinks.
4. Read: _Eye-Hand Behavior in Human-Robot Shared Manipulation_
  - Take-aways:
    - Eye gaze reveals trajectories (monitoring and planning)
5. Read: _Robot programming by Demonstration with Interactive Action Visualizations_
  - Takeaways:
    - This work is very relevant to the project I'm working on: Perform a single demonstration, and adjust the demonstration through feedback.
## April 20th 
1. "Finished" gesture recognition classifier (95% accuracy)
2. Starting on ROS2 design

## Meeting April 21st
- Write down similarities vs. differences for the papers read
- Check robustness of gesture classification and make more robust if necessary
- Add depth to gesture demonstration (relative) 
- Set up ROS2 + PyBullet + all inputs
- Test with friends

## April 21st
Today my focus will be on getting PyBullet and ROS2 up and running. This consists of:
- Understanding PyBullet better (URDF/joints etc)
- Creating ROS2 architecture
- Loading Robot Arm and replicating behavior

## April 22nd
- Started with ROS2 architecture for simulation and implementation of 
  - `gesture_publisher.py`: responsible for classifiying gestures and publishing demonstration coordinates
  - `sim_subscriber.py`: responsible for listening to gesture commands, coordinates and voice commands

## April 23rd
- Goal for this weekend is to finish ROS2 + PyBullet simulation control for gesture/movement/voice

## April 24th
- Got coordinate communication working (by making a fist)
- Implementing voice recog with VOSK
  - VOSK installation via pip gives error: "portaudio.h", fixed with `sudo apt install portaudio19-dev python3-pyaudio` on Ubuntu 20.04. 
- Voice recognition complete
- Gesture recognition complete
In summary, so far, all communcation is working (voice, gesture, coordination). The actual response to this feedback has not been implemented yet.

## April 25th
Today's goals:
- Refractor code - make it more modular, as right now the gesture publisher has 500 locs
- Implement saving, replaying of gestures on command (voice)
- Implement feedback tuning

## April 26th
Was unable to complete (2) and (3) yesterday. Today, I will focus on completing those. However, first, a launch file will be created to launch all the nodes as it's quite tedious to run each node individually.
- Saving and replaying of demonstration implemented (using voice commands)
- Feedback tuning partly implemented

## April 27th
- Voice rec is a bit weak when there is a background noise... Should I create my own model with limited classes? That might help... But I'm not familiar with voice rec models...
- Added directional as well as scaling feedback 
- Added faster + slower feedback
- Possible TODO: create extra thread for the replay feature - allowing the program to break off the replay if a ''stop' command is heard

## May 2nd
- Add a no voice launcher
- Add keyboard support

## May 3th
- Set up a barebone pepper simulator (nothing really works yet)

## May 5th
- Study PyBullet
- IK still not behaving as expected for pepper - opened an issue
## May 6th
- Will try to get Text to Speech working today, only a few hours available today
- Text2Speech working - partially implemented the interfacing. Also ordered a wearable mic.

## May 7th
Today:
- Improve interfacing with the simulator
- Got new mic working. After some messing around, with the use of
`pyaudio_test.py` script, one can retrieve the mic input to use for PyAudio in the file `speech_talker.py`
- Created issue asking for help on QiBullet for IK - no answers.

## May 8th - May 9th
- Add multiple loop (3 times for now) for replay
- Had some troubles where I spent a lot of time trying to get the external mic to be detected [fixed: mic input source should be set to 'pulse' on Ubuntu 20.04]
- Implemented a two keyword voice command detection: commands are processed only if they start with "hey". Example: "Hey record"

## May 10th
- TODO: Incorporate full body pose from mediapipe for depth detection
- Check out some repositories for IK and see if I can incorporate them in PyBullet / QiBullet. Seems like the way to go is to calculate angles and pass it to `setAngles` method.

## May 11th
- Pose is being detected correctly
- Tooling around with a repo that converts absolute coords (from origin frame) to angles for pepper
- Have not been able to get it completely correct but seems promising

## May 12
- It seems that pose detection and Hand detection reduces FPS tremendously, will have to see if that has adverse effects for recording the trajectory (likely). 
### Meeting with Dr. Baraka
- Gesture for feedback (possibly the main way if speech doesn't improve)
  - Google speech?
- Both pepper / robot arm and possiby testing which one is easier to use
- Trajectory with wrist, translate map to other joints

## May 13
- Pose detection and hand detection enabled, min(fps) = 11
- holistic, min(fps) = 15
- hand detection, min(fps) = 17
Ended up implementing hand + depth on the robot arm

## May 14th
- Pepper imitation is finally working
- Tomorrow:
  - will work on gestures, 
  - solidifying 'fist' detection for imitation, 
  - fine tuning the imitation movement(by using getting pepper's position and adding the relative positions to it)

## May 15th
- Added thousands of training points and improved gesture recognition
Forgot to mention yesterday: have to also add voice control sub module to this new package (pose\_simulator), as well as a new launch file (easy)
How should I approach switching modes with gestures. Also should voice recog be improved further? Needs more testing.
- Finished adding voice control to `pose_simulator` package and launch file

## May 16th
- Changed speech to  text to non blocking speech to text (gTTS)
- Fix bug where left fist would also set off movement
- Add trajectory lines when (recording) movement has been initiated
  - have a feedback command which shows what the current trajectory looks like
  - have the (new and old) trajectories be shown when feedback has been given
  - add text to show new trajectory 


## May 17-18th 
- Working non-stop on LSTM sign recognition
- LSTM fully working, still needs extra training data but it is working and partially implemented

## May 19th
- Implement some sign language mode changes

## May 20th
- Trained a new lstm classifier (incl. data) -- however this system can still be improved (by adding or removing classes, by changing the gestures for the sign language, and of course by simply growing the data set and training it)
- Got stuck trying to process callbacks outside of the main thread. This was simply solved by manually
calling `Node.spin_once`... But took better half of my time today.
- Implemented visuals to show mode changes on webcam feedback
- Hopefully can implement a robust sign language system today, max tomorrow.
Possible ideas to improve ROS architecture:
- use services to change mode (response being succesful feedback change or not, which can show the mode change - not having to rely on listening to a stream of topic as it is now)
- keep topic for imitating since that is a stream that constantly needs to be monitored
- use action for replay, allowing hte replay to be pre-emptively canceled

## May 21st-22nd
Spent most of the weekend experimenting with different sign language models / datasets 

## May 23
- Recorded new data set which has the best accuracy thus far
- Possibly going to expand the sign language model (more classes) as well as changing some gestures to improve classification accuracy. Currently there are 11 signs - thinking of adding 'bigger'/'smaller' to them as tthese are not in there currently. 
- Started writing the new architecture using services (see the idea on may 20th) 

## May 24
- Got services working and wrote a custom srv message for it -- but seems like it is impossible to share an object between two ros nodes running from two different ROS processes, therefore the whole idea of using the response from the service cannot be implemented, without the use of topics. Worked on this the whole morning/afternoon and will not ditch the idea.
- Plus side: learned a lot

## May 25th
- Created custom message nao\_move\_interfaces/msg/BotState.msg
- Added new mode: _move_ which allows the end user to move the robot 

## May 26th
- Robot arm had stopped working, fixed that and added debug lines to it as well
- Created a single message `WristCoordinates.msg` that has coordinates for both Pepper and Jaco (robot arm) 

## May 29th
- Fixed the rotation by translating to previous centroid -- by changing one data type to make the code smoother I got stuck with a bug for 5 hours

## May 30th
- Created methods to save trajectories and load trajectories (`Tester` class)
- Started on a gui to support the testing framework for the experiment

## May 31st
- Spent the morning setting up gui communication to work with simulator (able to draw template in simulator through gui now)
- Tested shape circle, accuracy was not the best as expected -- but the concept works
## June 8th 
Spent the last week studying and completing my last exam of the bachelor (took place yesterday). Will ease back into working on the thesis today.
- Read article Non verbal behavior modeling for Socially Assistive Robots and took notes (see `docs/related works`
- Made an attempt at reducing jaggedness in imitated movement by averaging out the last three coordinates - but it made the movement too laggy.

## June 9th
- Created Triangle shape and added it to the GUI
- GUI functionality extended 

## June 10th
- Extended the BotState message with subject\_name, test\_shape which can be displayed and manipulated via
the GUI

## June 11th
- Expanded GUI (shape manipulation)
- Created a ROS msg for gui communication (publisher)

## June 12th
- Experimented with adding objects in PyBullet for testing (i.e, throw a ball into a bucket)
- Added GUI functionality for it
- Attempted to create a new classifier with 'move' class in it, it's not performing great 

## June 13th
- Pilot study with Marina & Bahadir & Mariana
- Added timer to record mode 
- Added feedback on what to sign next
- Added new class in sign language 'move', increased the dataset to 215 video fragments per class (12 classes)

## June 14th
- Created an interface which the tester (me) has control over. This interface will show the sign languages and is controlled by the GUI
###Meeting notes:
- Create a reference object which moves along the template trajectory
  - The object will start moving when the hand is within a fixed distance of the object
  - MSE computed on the reference and the hand's coordinate
- Open anded plus suggestive questions for the subjective questionnaire (i.e, would this feature or that feature be liked etc)

## June 15th
- Expanded testing framework to start moving a (multi threaded) marker (ball) around the template trajectory when the hand coordinates are within vicinity (will be the start of the trajectory test) and the whole testing procedure will be initiated from the GUI. 
- The testing framework also records the position of the hand when the marker has started moving
- Only the square has been implemented today - the circle and triangle will be done tomorrow but the groundwork has been laid
- Moreover, I'll add automatic time recording when the test has been initiated and the ball marker has finished traversing the trajectory
- Also added functionality to turn depth on and off in the simulator via the GUI

## June 16th
- Automatic time recording implemented but also kept the manual just in case somebody is unable to do the full test run
- Square and triangle testing with marker and ball have been fully implemented
- Added some functionality for proper saving of experiment results, as well as recording the parameters (such as depth) when the test was recorded

## June 17th
- Finally finished implementing correct (link) movement imitation when the base has been rotated and translated, silly math mishap
- Added a marker on the ground for indicating where to move the base to
- Expanded GUI, automatic MSE calculation (and saving) when trajectory is complete
- Added countdown on camera feed when initiated recording (3s)

