import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
import sys
import errno
import pybullet as p
import pybullet_data
import time
from time import sleep
import math
from math import pi
import numpy as np
import csv

#pybullet configuration
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(False)
p.setGravity(0, 0, -10)
# load arm
model_path = "src/arm_simulator/arm_simulator/objects/jaco_robotiq_object.urdf"
arm_id = p.loadURDF(model_path, [0, 0, 0], useFixedBase=True)
angle = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
#p.loadURDF("/home/rakeshj/catkin_ws/src/sandbox/simulation/trajectory.urdf", [0,-1,1], angle)
# variables specific to this arm
eef_index = 8
num_joints = 18
rp = [-pi / 2, pi * 5 / 4, 0., pi / 2, 0., pi * 5 / 4, -pi / 2, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
ll = [-pi] * num_joints
ul = [pi] * num_joints
jd = [0.000001] * num_joints
jr = np.array(ll) - np.array(ul)
ik_solver = p.IK_DLS
 # configure arm
for i in range(num_joints):
    p.resetJointState(arm_id, i, rp[i])
    p.enableJointForceTorqueSensor(arm_id, i)

# extract joint names
joint_id = {}
for i in range(p.getNumJoints(arm_id)):
    jointInfo = p.getJointInfo(arm_id, i)
    joint_id[jointInfo[1].decode('UTF-8')] = jointInfo[0]

# adjust block mass
weight_joint_id = joint_id["weight_joint"]
p.changeDynamics(arm_id, weight_joint_id, mass=0.5)

controlled_joints = list(range(eef_index-1))
num_controlled_joints = eef_index-1
orn = p.getQuaternionFromEuler([math.pi / 2, math.pi / 2, math.pi / 2])
targetVelocities = [0] * num_controlled_joints
forces = [500] * num_controlled_joints
positionGains = [0.03] * num_controlled_joints
velocityGains = [1] * num_controlled_joints


class SimSubscriber(Node):
  def __init__(self):
    super().__init__('sim_subscriber')
    self.coords_subscription = self.create_subscription(
        String, 
        'movement_coords', 
        self.coords_callback, 
        10)
    self.coords_subscription # prevent unused variable warning
    self.gestures_subscription = self.create_subscription(
        String, 'gestures', self.gestures_callback, 10)
    self.gestures_subscription
    self.speech_subscription = self.create_subscription(
        String,
        'speech',
        self.speech_callback,
        10)
    self.speech_subscription

  def gestures_callback(self, msg):
    self.get_logger().info('Incoming gesture: "%s"' % msg.data)

  def coords_callback(self, msg):
    #self.get_logger().info('I heard: "%s"' % msg.data)
    pos = msg.data.split(',')
    type(pos[0])
    arrPos = [float(pos[0]),float(pos[1]),float(pos[2])]

    p.stepSimulation()
    jointPoses = p.calculateInverseKinematics(arm_id, eef_index, arrPos, orn,
                                                lowerLimits=ll,
                                                upperLimits=ul,
                                                jointRanges=jr,
                                                restPoses=rp,
                                                jointDamping=jd,
                                                solver=ik_solver)[:num_controlled_joints]
    p.setJointMotorControlArray(bodyIndex=arm_id,
                                jointIndices=controlled_joints,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=jointPoses,
                                targetVelocities=targetVelocities,
                                forces=forces,
                                positionGains=positionGains,
                                velocityGains=velocityGains)

  def speech_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
  rclpy.init(args=args)

  sim_subscriber = SimSubscriber()
  rclpy.spin(sim_subscriber)

  minimal_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
    main()
