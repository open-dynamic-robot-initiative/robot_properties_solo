import numpy as np
from math import pi
import rospkg
from os.path import join
import pinocchio as se3


class QuadrupedConfig:
    # name that is used by every other entities. I do not exactly why...
    robot_name = "quadruped"

    # PID gains
    kp = 5.0
    kd = 0.1
    ki = 0.0

    # here we use the same urdf as for the quadruped but without the freeflyer
    urdf_path = (
        join(rospkg.RosPack().get_path("robot_properties_quadruped"),
             "urdf",
             "quadruped.urdf")
    )

    yaml_path = (
        join(rospkg.RosPack().get_path("robot_properties_quadruped"),
             "config",
             "quadruped.yaml")
    )

    # pinocchio model
    robot_model = se3.buildModelFromUrdf(urdf_path,
                                         se3.JointModelFreeFlyer())

    # the number of motors, here they are the same as there are only revolute
    # joints
    nb_joints = robot_model.nv - 6

    # control time period
    control_period = 0.001
    dt = control_period

    # The inertia of a single blmc_motor
    motor_inertia = 0.045

    # The Kt constant of the motor [Nm/A]: tau = I * Kt
    motor_torque_cosntant = 0.025

    # maxCurrent = 12 # Ampers
    max_current = 2

    # maximum torques
    max_torque = motor_torque_cosntant * max_current

    # maximum control one can send, here the control is the current.
    max_control = max_current

    # mapping between the ctrl vector in the device and the urdf indexes
    urdf_to_dgm = (0,1,2,3,4,5,6,7)

    # ctrl_manager_current_to_control_gain I am not sure what it does so 1.0.
    ctrl_manager_current_to_control_gain=1.0

    map_joint_name_to_id = {}
    map_joint_limits = {}
    for i, (name, lb, ub) in enumerate(zip(robot_model.names[1:],
                                       robot_model.lowerPositionLimit,
                                       robot_model.upperPositionLimit)):
        map_joint_name_to_id[name] = i
        map_joint_limits[i] = [float(lb), float(ub)]

    max_qref = pi
