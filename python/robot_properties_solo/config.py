import numpy as np
from math import pi
import rospkg
from os.path import join, dirname
from os import environ
import pinocchio as se3
from pinocchio.utils import zero


class SoloConfig:
    # name that is used by every other entities. I do not exactly why...
    robot_name = "solo"

    # PID gains
    kp = 5.0
    kd = 0.1
    ki = 0.0

    # here we use the same urdf as for the quadruped but without the freeflyer
    urdf_path = (
        join(rospkg.RosPack().get_path("robot_properties_" + robot_name),
             "urdf",
             robot_name + ".urdf")
    )

    meshes_path = [
      dirname(rospkg.RosPack().get_path("robot_properties_" + robot_name))
    ]

    yaml_path = (
        join(rospkg.RosPack().get_path("robot_properties_" + robot_name),
             "config",
             "dgm_parameters.yaml")
    )

    # The inertia of a single blmc_motor
    motor_inertia = 0.0000045

    # The motor gear ratio
    motor_gear_ration = 9.

    # The Kt constant of the motor [Nm/A]: tau = I * Kt
    motor_torque_constant = 0.025

    # pinocchio model
    robot_model = se3.buildModelFromUrdf(urdf_path,
                                         se3.JointModelFreeFlyer())
    robot_model.rotorInertia[6:] = motor_inertia
    robot_model.rotorGearRatio[6:] = motor_gear_ration

    base_name = robot_model.frames[2].name

    # the number of motors, here they are the same as there are only revolute
    # joints
    nb_joints = robot_model.nv - 6

    # control time period
    control_period = 0.001
    dt = control_period


    # maxCurrent = 12 # Ampers
    max_current = 2

    # maximum torques
    max_torque = motor_torque_constant * max_current

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


    # Define the initial state.
    initial_configuration = [0.2,0,0.4, 0,0,0,1] + 4*[0.8,-1.6]
    initial_velocity = (8+6)*[0,]

    q0 = zero(robot_model.nq)
    q0[:] = np.asmatrix(initial_configuration).T
    v0 = zero(robot_model.nv)
    a0 = zero(robot_model.nv)

    @staticmethod
    def buildRobotWrapper():
        return se3.robot_wrapper.RobotWrapper(SoloConfig.robot_model)

    def joint_name_in_single_string(self):
        joint_names = ""
        for name in self.robot_model.names[2:]:
            joint_names += name + " "
        return joint_names