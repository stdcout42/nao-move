# nao-move: implementing multi-modal control on Nao/Pepper robots using ROS
## docs
- [raw logbook](https://github.com/stdcout42/nao-move/blob/main/docs/raw_logbook.md)
- [summarized logbook](https://github.com/stdcout42/nao-move/blob/main/docs/logbook.md)


## Google Drive
[Link](https://drive.google.com/drive/folders/15IWhDwY0hzxDpBRG4-ujRWeK-qrg7z2i?usp=sharing)

- results / data folder
    - Robot arm simulator demo [video](https://drive.google.com/file/d/1QnhcysKL1fhK-SB1o6wS4MHmiphN7qrs/view)

## collect_data.py
Helper file to record and save training/validation/test images
- Deprecates with keypoint hand-gesture-recognition-using-mediapipe library

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
