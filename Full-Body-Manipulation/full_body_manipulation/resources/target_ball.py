import pybullet as p
import os


class TargetBall:
    def __init__(self, client, startPos):
        f_name = os.path.join(os.path.dirname(__file__), 'target_ball.urdf')
        self.client = client
        self.ball = p.loadURDF(fileName=f_name,
                        basePosition=startPos,
                        physicsClientId=client)
        
    def get_obs(self):
        pos = p.getBasePositionAndOrientation(self.ball, physicsClientId=self.client)[0]
        vel = p.getBaseVelocity(self.ball, physicsClientId=self.client)[0]

        return [pos, vel]


    def getIds(self):
        return self.client, self.ball
