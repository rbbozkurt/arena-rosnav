from warehouse.envs.rware_node import Agent, AgentWarehouse
import gym

from gym import spaces
import numpy as np

class CustomEnv(gym.Env):

    def __init__(self):
        self.warehouse = AgentWarehouse()
        self.action_space = spaces.Discrete(6)
        self.warehouse.parse_string_to_map("0,A_2,0,0,0,0,0,0/0,0,0,0,0,0,0,0/0,0,S,0,0,0,0,0/0,0,0,0,0,0,0,0/0,G,0,0,0,0,0,0")
        print("test")
        print(self.warehouse.map_str)

        self.observation_space = spaces.Box(np.array([0]),np.array([20]),dtype=np.int8)
    
    def reset(self):
        '''
        print("RESETTTTTTTTTTTTTTTT'TTTT")
        t = self.warehouse.agent_dict
        for key,val in list(self.warehouse.agent_dict.items()):
            print("here")
            del self.warehouse.agent_dict[key]

        print(self.warehouse.agent_dict)
        del self.warehouse.agent_dict
        del self.warehouse.free_shelves
        del self.warehouse.shelf_dict
        del self.warehouse.goal_dict
        self.warehouse.map_str = ""
        '''
        del self.warehouse
        self.warehouse = AgentWarehouse()
        self.warehouse.parse_string_to_map("0,A_2,0,0,0,0,0,0/0,0,0,0,0,0,0,0/0,0,S,0,0,0,0,0/0,0,0,0,0,0,0,0/0,G,0,0,0,0,0,0")
        obs = self.warehouse.observe()
        return obs
    
    def step(self,action):
        self.warehouse.debug_agents_actions(action)
        obs = self.warehouse.observe()
        reward = self.warehouse.evaluate()
        done = self.warehouse.is_done()

        return obs, reward, done , {}

    def render(self):
        self.warehouse.view()

    