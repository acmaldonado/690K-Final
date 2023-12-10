import gymnasium as gym
import stable_baselines3
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import time
from importlib.machinery import SourceFileLoader

environment = SourceFileLoader("environment", '/home/andy/Desktop/pybullet-test/Full-Body-Manipulation/full_body_manipulation/envs/full_body_panda_env.py').load_module()

def main():
    gym.register('FullBodyPanda-v0', environment.FullBodyPanda)

    # vec_env = DummyVecEnv([lambda: gym.make('FullBodyPanda-v0', max_episode_steps = 1000)])
    # vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True)
    vec_env = gym.make('FullBodyPanda-v0', max_episode_steps = 1000)
    model = stable_baselines3.A2C("MultiInputPolicy", vec_env, verbose=1, tensorboard_log="/home/andy/Desktop/pybullet-test/tensor_logs")
    model.learn(total_timesteps=100000)

    vec_env = model.get_env()
    obs = vec_env.reset(None)

    for i in range(1000):
        action, _state = model.predict(obs, deterministic=True)
        obs, reward, done, info = vec_env.step(action)

if __name__ == '__main__':
    main()