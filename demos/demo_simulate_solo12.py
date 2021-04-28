#!/usr/bin/env python

"""demo_simulate_solo12

Simple demo showing how the simulation setup works.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import time
import numpy as np
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config

if __name__ == "__main__":

    # ! Create a Pybullet simulation environment before any robots !
    env = BulletEnvWithGround()

    # Create a robot instance. This adds the robot to the simulator as well.
    robot = Solo12Robot()

    # Add the robot to the env to update the internal structure of the robot
    # ate every simulation steps.
    env.add_robot(robot)

    # Some control.
    tau = np.zeros(robot.nb_dof)

    # Reset the robot to some initial state.
    q0 = np.matrix(Solo12Config.initial_configuration).T
    dq0 = np.matrix(Solo12Config.initial_velocity).T
    robot.reset_state(q0, dq0)

    # Run the simulator for 2000 steps
    for i in range(2000):
        # TODO: Implement a controller here.
        robot.send_joint_command(tau)

        # Step the simulator.
        env.step(
            sleep=True
        )  # You can sleep here if you want to slow down the replay

    # Read the final state and forces after the stepping.
    q, dq = robot.get_state()
    active_eff, forces = robot.get_force()
    print("q", q)
    print("dq", dq)
    print("active eff", active_eff)
    print("forces", forces)
