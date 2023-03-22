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
from bullet_utils.wrapper import PinBulletWrapper
from robot_properties_solo.config import Solo8Config

dt = 1e-3


class Solo8Robot(PinBulletWrapper):
    def __init__(
        self,
        pos=None,
        orn=None,
        useFixedBase=False,
        init_sliders_pose=4
        * [
            0,
        ],
    ):

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.40]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(Solo8Config.resources.package_path)
        self.urdf_path = Solo8Config.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos,
            orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=useFixedBase,
        )
        pybullet.getBasePositionAndOrientation(self.robotId)

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

        self.slider_a = pybullet.addUserDebugParameter(
            "a", 0, 1, init_sliders_pose[0]
        )
        self.slider_b = pybullet.addUserDebugParameter(
            "b", 0, 1, init_sliders_pose[1]
        )
        self.slider_c = pybullet.addUserDebugParameter(
            "c", 0, 1, init_sliders_pose[2]
        )
        self.slider_d = pybullet.addUserDebugParameter(
            "d", 0, 1, init_sliders_pose[3]
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

    def get_slider_position(self, letter):
        if letter == "a":
            return pybullet.readUserDebugParameter(self.slider_a)
        if letter == "b":
            return pybullet.readUserDebugParameter(self.slider_b)
        if letter == "c":
            return pybullet.readUserDebugParameter(self.slider_c)
        if letter == "d":
            return pybullet.readUserDebugParameter(self.slider_d)

    def reset_to_initial_state(self) -> None:
        """Reset robot state to the initial configuration (based on Solo8Config)."""
        q0 = np.matrix(Solo8Config.initial_configuration).T
        dq0 = np.matrix(Solo8Config.initial_velocity).T
        self.reset_state(q0, dq0)
