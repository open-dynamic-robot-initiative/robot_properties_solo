"""solo8wrapper

Solo8 pybullet interface using pinocchio's convention.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import numpy as np
import time
import os
import pybullet
from pinocchio_bullet.wrapper import PinBulletWrapper
from robot_properties_solo.config import Solo8Config

dt = 1e-3


class Solo8Robot(PinBulletWrapper):

    def __init__(self, pos=None, orn=None):

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.40]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        self.urdf_path = Solo8Config.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos, orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=False,
        )

        # Create the robot wrapper in pinocchio.
        self.pin_robot = Solo8Config.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.base_link_name = "base_link"
        controlled_joints = []
        for leg in ["FL", "FR", "HL", "HR"]:
            controlled_joints += [leg + "_HFE", leg + "_KFE"]
        self.joint_names = controlled_joints

        # Creates the wrapper by calling the super.__init__.
        super(Solo8Robot, self).__init__(
            self.robotId,
            self.pin_robot,
            controlled_joints,
            ["FL_ANKLE", "FR_ANKLE", "HL_ANKLE", "HR_ANKLE"],
        )

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)
