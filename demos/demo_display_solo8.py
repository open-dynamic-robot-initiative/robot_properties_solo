#!/usr/bin/env python

"""demo_display_solo8

Basic loading and visualization for the Solo8 robot using gepetto viewer.

license: BSD 3-Clause License
copyrights: Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""


def with_gepetto_gui_helper():
    import time
    from robot_properties_solo.config import Solo8Config
    from gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene
    from gepetto_gui_helper.robot_visual import RobotVisual
    from gepetto_gui_helper.frame import Frame

    gepetto_gui_scene = GepettoGuiScene("solo_scene", "solo_window")
    config = Solo8Config()
    solo_visual = RobotVisual(
        gepetto_gui_scene, "solo", config.urdf_path, config.meshes_path
    )
    solo_visual.display(config.q0)
    # place the world frame
    world_frame = Frame(gepetto_gui_scene)

    # Example of moving the robot forward and updating the display every time.
    q = config.q0.copy()
    for i in range(10):
        q[0] += 0.05
        solo_visual.display(q)
        time.sleep(0.2)


def with_pinocchio_wrapper():
    import numpy as np
    import pinocchio as se3
    import time
    import os

    from robot_properties_solo.config import Solo8Config

    # Load the robot urdf.
    robot = Solo8Config.buildRobotWrapper()

    # Setup the display (connection to gepetto viewer) and load the robot model.
    robot.initDisplay(loadModel=True)

    # Create a first initial position for the robot. Both legs are bent inwards.
    q = np.matrix(Solo8Config.initial_configuration).T

    # Display the configuration in the viewer.
    robot.display(q)

    # Example of moving the robot forward and updating the display every time.
    for i in range(10):
        q[0] += 0.05
        robot.display(q)
        time.sleep(0.2)


if __name__ == "__main__":

    with_pinocchio_wrapper()
