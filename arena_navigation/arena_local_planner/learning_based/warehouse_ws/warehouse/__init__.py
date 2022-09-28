
from gym.envs.registration import register
from warehouse.envs import custom_env

register(
    id='Warehouse-test',
    entry_point='warehouse.envs:CustomEnv',
    max_episode_steps=2000
)