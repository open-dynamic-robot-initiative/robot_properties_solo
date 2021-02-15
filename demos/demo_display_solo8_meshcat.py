#!/usr/bin/env python

"""demo_display_solo8_meshcat

Basic loading and visualization for the Solo8 robot using meshcat visualizer.

license: BSD 3-Clause License
copyrights: Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
import pinocchio as pin
import time
import os
import meshcat

from robot_properties_solo.config import Solo8Config

if __name__ == "__main__":

    # Load the robot urdf.
    robot = Solo8Config.buildRobotWrapper()

viz = pin.visualize.MeshcatVisualizer(
    robot.model, robot.collision_model, robot.visual_model
)

try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

viz.loadViewerModel()
q = np.matrix(Solo8Config.initial_configuration).T

viz.display(q)

# Example of moving the robot forward and updating the display every time.
for i in range(500):
    q[0] += 0.002
    viz.display(q)
    time.sleep(0.01)
