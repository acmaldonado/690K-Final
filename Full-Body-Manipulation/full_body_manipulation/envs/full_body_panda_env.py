import gym
import numpy as np
import pybullet as p
import os
import time

from pathlib import Path
from importlib.machinery import SourceFileLoader

# from full_body_manipulation.resources.panda import Panda
# from full_body_manipulation.resources.plane import Plane
# from full_body_manipulation.resources.target_ball import TargetBall

panda = SourceFileLoader("panda", "/home/andy/Desktop/pybullet-test/Full-Body-Manipulation/full_body_manipulation/resources/panda.py").load_module()
plane = SourceFileLoader("plane", "/home/andy/Desktop/pybullet-test/Full-Body-Manipulation/full_body_manipulation/resources/plane.py").load_module()
target_ball = SourceFileLoader("target_ball", "/home/andy/Desktop/pybullet-test/Full-Body-Manipulation/full_body_manipulation/resources/target_ball.py").load_module()


class FullBodyPanda(gym.Env):
    metadata = {'render.modes': ['human']}

    X_BOUND = [-1.5, 1.5]
    Y_BOUND = [-1.5, 1.5]
    Z_BOUND = [0, 2]

    MAX_STEPS = 1000

    PANDA_START_POS = [0,0,0]
    PANDA_START_ORIENTATION = p.getQuaternionFromEuler([0,0,0])

    CONNECT_MODE = p.GUI
    #CONNECT_MODE = p.DIRECT

  
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

        self.panda = None
        self.object = None
        self.goal = None
        self.done = False
        self.steps = 0

        self.reset()

    def get_combined_obs(self):
        panda_obs = self.panda.get_partial_observation()
        obj_obs = self.object.get_obs()

        obs = np.concatenate((panda_obs,obj_obs,self.goal),axis=None)

        return obs
    
    def reward_function(self):
        dist = -np.linalg.norm(np.array(self.object.get_obs()[0]) - np.array(self.goal))
        return dist, False

    def step(self, action):
        self.steps += 1
        self.panda.apply_action(action)
        p.stepSimulation()
        curr_obs = self.get_combined_obs()

        reward, self.done = self.reward_function()

        tlimit_reached = self.steps >= self.MAX_STEPS

        return curr_obs, reward, self.done, tlimit_reached, dict()


    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10)
        plane.Plane(self.client)
        self.panda = panda.Panda(self.client, self.PANDA_START_POS, self.PANDA_START_ORIENTATION)

        #Randomly generates a starting object position and goal position

        obj_start_pos = [
            self.np_random.uniform(self.X_BOUND[0], self.X_BOUND[1]),
            self.np_random.uniform(self.Y_BOUND[0], self.Y_BOUND[1]),
            self.np_random.uniform(self.Z_BOUND[0], self.Z_BOUND[1])
        ]

        self.goal = [
            self.np_random.uniform(self.X_BOUND[0], self.X_BOUND[1]),
            self.np_random.uniform(self.Y_BOUND[0], self.Y_BOUND[1]),
            0.1
        ]
        self.done = False

        print(f'Reset object position: {obj_start_pos}\nReset goal position: {self.goal}')

        #Instantiate target object and visual representation of goal

        self.object = target_ball.TargetBall(self.client, obj_start_pos)

        f_name = str(Path(os.path.dirname(__file__)).parent.joinpath('resources/goal_visual.urdf'))
        print(f'Loading goal from {f_name} ...')
        p.loadURDF(f_name, self.goal, physicsClientId=self.client, useFixedBase=1)

        self.steps = 0

        return self.get_combined_obs()


    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)
        
    def seed(self, seed=None): 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

if __name__ == "__main__":
    #Load test of environment
    env = FullBodyPanda()
    for episodes in range(20):
        for i in range(env.MAX_STEPS):
            p.stepSimulation()
            print(f'Reward: {env.reward_function()}')
            time.sleep(1./240.)
        print(f'First Observation: {env.reset()}')
        print(f'First reward: {env.reward_function()}')
    env.close()
    