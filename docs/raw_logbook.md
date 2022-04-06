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

   

   
