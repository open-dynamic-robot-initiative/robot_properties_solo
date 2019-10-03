import numpy as np

import time

import os
import rospkg
import pybullet as p
import pinocchio as se3

from py_pinocchio_bullet.wrapper import PinBulletWrapper
from robot_properties_solo.config import Solo12Config


dt = 1e-3

class Quadruped12Robot(PinBulletWrapper):
    @staticmethod
    def initPhysicsClient():
        physicsClient = p.connect(p.GUI)
        p.setGravity(0,0, -9.81)
        p.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
        return physicsClient
    
    def __init__(self, physicsClient=None):
        if physicsClient is None:
            self.physicsClient = self.initPhysicsClient()

        # Load the plain.
        plain_urdf = (rospkg.RosPack().get_path("robot_properties_solo") +
                      "/urdf/plane_with_restitution.urdf")
        self.planeId = p.loadURDF(plain_urdf)

        # Load the robot
        robotStartPos = [0.,0,0.40]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

        self.urdf_path = Solo12Config.urdf_path
        self.robotId = p.loadURDF(self.urdf_path, robotStartPos,
            robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=False)
        p.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path)) + '/urdf']
        self.pin_robot = Solo12Config.buildRobotWrapper()

        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)

        for ji in range(num_joints):
            p.changeDynamics(self.robotId, ji, linearDamping=.04,
                angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        self.base_link_name = "base_link"
        controlled_joints = []
        for leg in ['FL', 'FR', 'HL', 'HR']:
            controlled_joints += [leg + '_HAA', leg + '_HFE', leg + '_KFE']
        self.joint_names = controlled_joints
        

        # Creates the wrapper by calling the super.__init__.
        super(Quadruped12Robot, self).__init__(self.robotId, self.pin_robot,
            controlled_joints,
            ['FL_ANKLE', 'FR_ANKLE', 'HL_ANKLE', 'HR_ANKLE']
        )
        
    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError('Need to provide q and dq or non of them.')
        
        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)
        
        
if __name__ == "__main__":
    # Create a robot instance. This initializes the simulator as well.
    robot = Quadruped12Robot()
    tau = np.zeros(12)

    # Reset the robot to some initial state.
    q0 = np.matrix(Solo12Config.initial_configuration).T
    dq0 = np.matrix(Solo12Config.initial_velocity).T
    robot.reset_state(q0, dq0)


    # Run the simulator for 100 steps
    for i in range(230):
        # TODO: Implement a controller here.
        robot.send_joint_command(tau)
        
        # Step the simulator.
        p.stepSimulation()
        # time.sleep(0.001) # You can sleep here if you want to slow down the replay

    # Read the final state and forces after the stepping.
    q, dq = robot.get_state()
    active_eff, forces = robot.get_force()
    print('q', q)
    print('dq', dq)
    print('active eff', active_eff)
    print('forces', forces)
