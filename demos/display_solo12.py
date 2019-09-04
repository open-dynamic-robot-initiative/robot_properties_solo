""" Basic loading and visualization for the Solo robot using gepetto viewer. """

import numpy as np
import pinocchio as se3
import time
import os

from robot_properties_solo.config import Solo12Config

# Load the robot urdf.
robot = Solo12Config.buildRobotWrapper()

# Setup the display (connection to gepetto viewer) and load the robot model.
robot.initDisplay(loadModel=True)

# Create a first initial position for the robot. Both legs are bent inwards.
q = np.matrix(Solo12Config.initial_configuration).T

# Turn the legs outside
q[[10, 16]] = -0.5 # Right side of quadruped
q[[ 7, 13]] = 0.5  # Left side of quadruped

# Display the configuration in the viewer.
robot.display(q)

# Example of moving the robot forward and updating the display every time.
for i in range(10):
  q[0] += 0.05
  robot.display(q)
  time.sleep(0.2)
