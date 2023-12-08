from gym.envs.registration import register

register(
    id='FullBodyPanda-v0',
    entry_point='full_body_manipulation.envs:FullBodyPanda'
)