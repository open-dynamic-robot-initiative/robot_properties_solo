""" Basic loading and visualization for the Solo robot using gepetto viewer. """

import numpy as np
import pinocchio as se3
import time
import os

from robot_properties_solo.config import SoloConfig

# Load the robot urdf.
robot = SoloConfig.buildRobotWrapper()

# Setup the display (connection to gepetto viewer) and load the robot model.
robot.initDisplay(loadModel=True)

# Create a first initial position for the robot. Both legs are bent inwards.
q = robot.model.neutralConfiguration.copy()
q[0] = 0.2
q[2] = 0.3
q[7:11:2] = 0.8
q[11::2] = -0.8
q[8:12:2] = -1.6
q[12::2] = 1.6

# Display the configuration in the viewer.
robot.display(q)

# Example of moving the robot forward and updating the display every time.
for i in range(10):
  q[0] += 0.05
  robot.display(q)
  time.sleep(0.2)
