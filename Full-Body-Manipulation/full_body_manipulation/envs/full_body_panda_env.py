import gym
import numpy as np
import pybullet as p
class FullBodyPanda(gym.Env):
    metadata = {'render.modes': ['human']}

    X_BOUND = [-10, 10]
    Y_BOUND = [-10, 10]
    Z_BOUND = [0, 10]

    CONNECT_MODE = p.DIRECT

  
    def __init__(self):
        #Outputs torques between -1 and 1 for each non-fixed joint to be multiplied by max torque
        self.action_space = gym.spaces.Box(
            low=np.array([-1 for i in range(7)]),
            high=np.array([1 for i in range(7)]))
        #Observation space
        #   joint_pos: [Continuous 7x1] position of each joint; limits set according to panda URDF
        #   joint_vel: [Continuous 7x1] angular velocity of each joint; limits set according to panda URDF
        #   obj_pos: [Continuous 3x1] position of the object
        #   obj_vel: [Continuous 3x1] velocity of the object
        #   goal_pos: [Continuous 3x1] position of goal
        #   (optional)link_contact: [Binary 8] whether or not a link is in contact with the object

        self.observation_space = gym.spaces.Dict({
            "joint_pos": gym.spaces.Box(
                low=np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]),
                high=np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
            ),
            "joint_vel": gym.spaces.Box(
                low=np.array([-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100]),
                high=np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
            ),
            "obj_pos": gym.spaces.Box(
                low=np.array([self.X_BOUND[0], self.Y_BOUND[0], self.Z_BOUND[0]]),
                high=np.array([self.X_BOUND[1], self.Y_BOUND[1], self.Z_BOUND[1]])
            ),
            "obj_vel": gym.spaces.Box(
                low=np.array([-np.inf, -np.inf, -np.inf]),
                high=np.array([np.inf, np.inf, np.inf])
            ),
            "goal_pos": gym.spaces.Box(
                low=np.array([self.X_BOUND[0], self.Y_BOUND[0], self.Z_BOUND[0]]),
                high=np.array([self.X_BOUND[1], self.Y_BOUND[1], self.Z_BOUND[1]])
            )
            #, "link_contact": gym.spaces.MultiBinary(8)
        })

        self.np_random, _= gym.utils.seeding.np_random()

        self.client = p.connect(self.CONNECT_MODE)

    def step(self, action):
        pass

    def reset(self):
        pass

    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)
        
    def seed(self, seed=None): 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
