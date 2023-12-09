import pybullet as p
import numpy as np
import os


class TargetBall:
    def __init__(self, client, startPos):
        f_name = os.path.join(os.path.dirname(__file__), 'target_ball.urdf')
        self.client = client
        self.ball = p.loadURDF(fileName=f_name,
                        basePosition=startPos,
                        physicsClientId=client)
        
    def get_obs(self):
        pos = np.array(p.getBasePositionAndOrientation(self.ball, physicsClientId=self.client)[0])
        vel = np.array(p.getBaseVelocity(self.ball, physicsClientId=self.client)[0])

        return {"obj_pos":pos, "obj_vel":vel}


    def get_ids(self):
        return self.client, self.ball
