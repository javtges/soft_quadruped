from gym.envs.registration import register

register(
    id='hsa_robot-v0',
    entry_point='gym_hsa_robot.envs:HSARobot_Env',
)