import gymnasium as gym
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

    SPAWN_X_BOUND = [-1, 1]
    SPAWN_Y_BOUND = [-1, 1]
    SPAWN_Z_BOUND = [0, 0.5]

    MAX_STEPS = 10000
    DELTA_T = 1.0/240.0
    DELTA = 0.25

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
        self.has_touched = False
        self.steps = 0
        self.done = False

        #Reward Parameters

        # Positive rewards
        self.task_exp_scale = -3.0
        self.task_scale = 0.3
        self.task_offset = -0.31

        self.qdot_exp_scale = -0.01
        self.qdot_scale = 0.05
        self.qdot_offset = -0.05

        self.qddot_exp_scale = -20.0
        self.qddot_scale = 0.05
        self.qddot_offset = -0.05

        self.torque_exp_scale = -0.01
        self.torque_scale = 0.15
        self.torque_offset = -0.15

        self.dtorque_exp_scale = -0.01
        self.dtorque_scale = 0.2
        self.dtorque_offset = -0.2

        #Previous values for reward calculation
        self.last_qdot = np.zeros(7)
        self.last_command = np.zeros(7)

        self.reset(None)

    def get_combined_obs(self):
        panda_obs = self.panda.get_partial_observation()
        obj_obs = self.object.get_obs()

        panda_obs.update(obj_obs)
        panda_obs["goal_pos"] = self.goal

        return panda_obs
    
    def ball_out_of_bounds(self):
        obj_pos = self.object.get_obs()['obj_pos']
        if obj_pos[0] < self.X_BOUND[0] or obj_pos[0] > self.X_BOUND[1]:
            return 0
        if obj_pos[1] < self.Y_BOUND[0] or obj_pos[1] > self.Y_BOUND[1]:
            return 1
        if obj_pos[2] < self.Z_BOUND[0] or obj_pos[2] > self.Z_BOUND[1]:
            return 2
        return -1

    
    def reward_function(self, action=None):
        done = False
        total_reward = 0
        if action is None:
            action = np.zeros(7)
        #Calculate task reward
        goal_dist = np.linalg.norm(np.array(self.object.get_obs()['obj_pos']) - np.array(self.goal))
        r_task = self.task_scale*np.exp(self.task_exp_scale*np.square(goal_dist)) + self.task_offset
        #Check if goal has been reached. Add large positive reward for successful completion
        if goal_dist <= self.DELTA:
            done = True
            total_reward += 1000
            print('Ball was close to goal!')

        #Calculate qdot regularization reward
        curr_qdot = self.panda.get_partial_observation()['joint_vel']
        r_qdot = self.qddot_scale*np.exp(self.qddot_exp_scale* np.square(np.linalg.norm(curr_qdot))) + self.qdot_offset

        #Calculate qddot regularization reward by estimating qddot
        qddot_est = (curr_qdot - self.last_qdot)/self.DELTA_T
        r_qddot = self.qddot_scale*np.exp(self.qddot_exp_scale*np.square(np.linalg.norm(qddot_est))) + self.qddot_offset
        self.last_qdot = curr_qdot

        #Calculate torque regularization reward
        r_torque = self.torque_scale*np.exp(self.torque_exp_scale*np.linalg.norm(action)) + self.torque_offset

        #Calculate dtorque regularization reward
        r_dtorque = self.dtorque_scale*np.exp(self.dtorque_exp_scale*np.linalg.norm(action - self.last_command)) + self.dtorque_offset
        self.last_command = action

        # if not self.has_touched:
        #     contact_points = p.getContactPoints(self.panda.get_ids()[1], self.object.get_ids()[1])
        #     if len(contact_points) > 0:
        #         reward += 20
        #         print(f'Touched! Reward: {reward}')
        #         self.has_touched = True
        
        #Check boundary conditions for object. Add large penalty for out of bounds
        bound_check = self.ball_out_of_bounds()
        if bound_check >= 0:
            done = True
            total_reward -= 1000
            print(f'Ball was out of bound {bound_check}')

        #Sum all of the rewards
        total_reward += r_task + r_qdot + r_qddot + r_torque + r_dtorque

        return total_reward, done

    def step(self, action):
        self.steps += 1
        self.panda.apply_action(action)
        p.stepSimulation()
        curr_obs = self.get_combined_obs()

        reward, self.done = self.reward_function(action)

        trunc = self.steps >= self.MAX_STEPS

        return curr_obs, reward, self.done, trunc, dict()


    def reset(self, seed=None, options=None):
        if seed is not None:
            self.seed(seed)
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10)
        p.setTimeStep(self.DELTA_T, physicsClientId=self.client)
        plane.Plane(self.client)
        self.panda = panda.Panda(self.client, self.PANDA_START_POS, self.PANDA_START_ORIENTATION)

        #Randomly generates a starting object position and goal position

        obj_start_pos = [
            self.np_random.uniform(self.SPAWN_X_BOUND[0], self.SPAWN_X_BOUND[1]),
            self.np_random.uniform(self.SPAWN_Y_BOUND[0], self.SPAWN_Y_BOUND[1]),
            0.21
        ]

        self.goal = [
            self.np_random.uniform(self.X_BOUND[0], self.X_BOUND[1]),
            self.np_random.uniform(self.Y_BOUND[0], self.Y_BOUND[1]),
            0.1
        ]
        self.done = False
        self.steps = 0
        self.has_touched = False

        print(f'Reset object position: {obj_start_pos}\nReset goal position: {self.goal}')

        #Instantiate target object and visual representation of goal

        self.object = target_ball.TargetBall(self.client, obj_start_pos)

        f_name = str(Path(os.path.dirname(__file__)).parent.joinpath('resources/goal_visual.urdf'))
        print(f'Loading goal from {f_name} ...')
        p.loadURDF(f_name, self.goal, physicsClientId=self.client, useFixedBase=1)

        self.steps = 0

        return self.get_combined_obs(), dict()


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
    for episodes in range(5):
        for i in range(env.MAX_STEPS):
            obs, reward, done, trunc, info = env.step(np.array([0,0,0,0,0,0,0]))
            if done:
                break
            print(f'Reward: {reward}')
            time.sleep(1./240.)
        print(f'First Observation: {env.reset()}')
        print(f'First reward: {env.reward_function()}')
    env.close()
    