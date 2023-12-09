import pybullet as p
import numpy as np
from pathlib import Path
import os

class Panda:

    MAX_TORQUE = 500

    def __init__(self, client, startPos, startOrientation):
        self.client = client
        f_name = str(Path(os.path.dirname(__file__)).parent.parent.parent.joinpath('franka_panda_description-master/robots/panda_arm.urdf'))
        print(f'Trying to load Panda from {f_name} ...')
        self.panda = p.loadURDF(f_name, startPos, startOrientation, physicsClientId=client, useFixedBase=1, globalScaling=2)

        self.joints = [i for i in range(7)]
    
    def get_ids(self):
        return self.client, self.panda
    
    def apply_action(self, action):
        #Because action for each joint is between 1 and -1 we multiply it by the max allowable torque
        torque_values = action * self.MAX_TORQUE
        p.setJointMotorControlArray(self.panda, self.joints, controlMode=p.TORQUE_CONTROL, forces=torque_values, physicsClientId=self.client)

    def get_partial_observation(self):
        joint_states = p.getJointStates(self.panda, self.joints, physicsClientId=self.client)

        joint_pos = np.empty(7)
        joint_vel = np.empty(7)

        for i in range(len(joint_states)):
            joint_pos[i] = joint_states[i][0]
            joint_vel[i] = joint_states[i][1]

        return {"joint_pos":joint_pos, "joint_vel":joint_vel}

        