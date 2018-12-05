import numpy as np
from math import pi
import rospkg
from os.path import join
import pinocchio as se3


class QuadrupedConfig:
    # name that is used by every other entities. I do not exactly why...
    robotRef = "quadruped"

    # PID gains
    kp = 5.0
    kd = 1.0
    ki = 0.0

    # here we use the same urdf as for the quadruped but without the freeflyer
    testRobotPath = (
        join(rospkg.RosPack().get_path("robot_properties_quadruped"),
             "urdf",
             "quadruped.urdf")
    )

    testRobotYamlPath = (
        join(rospkg.RosPack().get_path("robot_properties_quadruped"),
             "config",
             "quadruped.yaml")
    )

    # pinocchio model
    robot_model = se3.buildModelFromUrdf(testRobotPath,
                                         se3.JointModelFreeFlyer())

    # the number of motors, here they are the same as there are only revolute
    # joints
    nbJoints = robot_model.nv - 6

    # control time period
    controlDT = 0.001

    # The inertia of a single blmc_motor
    motor_I = 0.045

    # The Kt constant of the motor [Nm/A]: tau = I * Kt
    motor_KT = 0.025

    # maxCurrent = 12 # Ampers
    maxCurrent = 2

    # maximum torques
    maxTorque = motor_KT * maxCurrent

    # maximum control one can send, here the control is the current.
    maxControl = maxCurrent

    # mapping between the ctrl vector in the device and the urdf indexes
    urdftosot = (0,1,2,3,4,5,6,7)

    # ctrlManagerCurrentToControlGain I am not sure what it does so 1.0.
    ctrlManagerCurrentToControlGain=1.0

    mapJointNameToID = {}
    mapJointLimits = {}
    for i, (name, lb, ub) in enumerate(zip(robot_model.names[1:],
                                       robot_model.lowerPositionLimit,
                                       robot_model.upperPositionLimit)):
        mapJointNameToID[name] = i
        mapJointLimits[i] = [float(lb), float(ub)]

    max_qref = pi
