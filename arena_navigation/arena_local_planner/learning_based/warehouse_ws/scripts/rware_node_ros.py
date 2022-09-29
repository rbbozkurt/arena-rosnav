#!/usr/bin/env python3
import string
from enum import Enum
from typing import Tuple, Optional
from std_msgs.msg import String
import numpy as np
import rospy

import math
import numpy as np


class Action(Enum):
    NONE = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    LOAD = 4
    UNLOAD = 5


# in clockwise
class Direction(Enum):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3


class RewardType(Enum):
    GLOBAL = 0
    INDIVIDUAL = 1
    TWO_STAGE = 2


class EntityType(Enum):
    SHELF = "S"
    GOAL = "G"
    AGENT = "A"
    NONE = "0"


ROW_SEPARATOR = '/'
COL_SEPARATOR = ','
FEATURE_SEPARATOR = '_'
FEATURE_ENTITY_IND = 0
FEATURE_ID_IND = 2
FEATURE_DIR_IND = 1


class Goal:
    counter = 0

    def __init__(self, y: int, x: int):
        Goal.counter += 1
        self.id = Goal.counter
        self.x = x
        self.y = y


class Shelf:
    counter = 0

    def __init__(self, y: int, x: int):
        Shelf.counter += 1
        self.id = Shelf.counter
        self.x = x
        self.y = y


class Agent:
    counter = 0

    def __init__(self, y: int, x: int, dir: Direction):
        Agent.counter += 1
        self.id = Agent.counter
        self.x = x
        self.y = y
        self.cur_dir = dir
        self.carrying_shelf: Shelf = None
        self.carrying_shelf_id: int = None
        self.has_delivered: bool = False
        self.goal = None
        self.des_action_on_goal = None

    def unload(self):
        if self.carrying_shelf:
            self.carrying_shelf = None
            self.carrying_shelf_id = None
            self.has_delivered = True
        return

    def load(self, shelf: Shelf):
        if self.carrying_shelf:
            return
        else:
            self.carrying_shelf = shelf
            self.carrying_shelf_id = shelf.id
            self.carrying_shelf.y = self.y
            self.carrying_shelf.x = self.x
            self.has_delivered = False
        return

    def step(self, action: Action):
        # check action
        if action == Action.LEFT:
            self.cur_act = Action.NONE
            if self.cur_dir == Direction.RIGHT:
                self.cur_dir = Direction.UP
            elif self.cur_dir == Direction.UP:
                self.cur_dir = Direction.LEFT
            elif self.cur_dir == Direction.LEFT:
                self.cur_dir = Direction.DOWN
            elif self.cur_dir == Direction.DOWN:
                self.cur_dir = Direction.RIGHT
        elif action == Action.RIGHT:
            self.cur_act = Action.NONE
            if self.cur_dir == Direction.RIGHT:
                self.cur_dir = Direction.DOWN
            elif self.cur_dir == Direction.DOWN:
                self.cur_dir = Direction.LEFT
            elif self.cur_dir == Direction.LEFT:
                self.cur_dir = Direction.UP
            elif self.cur_dir == Direction.UP:
                self.cur_dir = Direction.RIGHT
        elif action == Action.NONE:
            self.cur_act = action
        elif action == Action.FORWARD:
            self.cur_act = action
            if self.cur_dir == Direction.DOWN:
                self.y = self.y + 1
            elif self.cur_dir == Direction.UP:
                self.y = self.y - 1
            elif self.cur_dir == Direction.LEFT:
                self.x = self.x - 1
            elif self.cur_dir == Direction.RIGHT:
                self.x = self.x + 1

            if self.carrying_shelf:
                self.carrying_shelf.y = self.y
                self.carrying_shelf.x = self.x
        elif action == Action.LOAD:
            self.cur_act = Action.NONE
            self.load()
            if self.carrying_shelf:
                self.carrying_shelf.move(self.y, self.x)
        elif action == Action.UNLOAD and self.carrying_shelf:
            self.cur_act = Action.NONE
            self.unload()


class AgentWarehouse:

    def __init__(self):

        self.map_width = None
        self.map_height = None
        self.grid_width = None
        self.grid_height = None

        self.agent_dict = {}
        self.carrying_agent_dict = {}
        self.shelf_dict = {}
        self.goal_dict = {}
        self.map_str = None


        # publishers

        self.test_msg = rospy.Subscriber("map_topic",String, self.parse_string_to_map)
        
        self.update_map_pub = rospy.Publisher("update_map_topic",String,queue_size=10)

        self.agent_agents_sub = rospy.Subscriber("agent_action_topic", String, self.debug_agents_actions)
        self.spawn_shelf_sub = rospy.Subscriber('spawn_shelf_topic', String,self.debug_spawn_shelves)
        self.spawn_agent_sub = rospy.Subscriber('spawn_agent_topic', String,self.debug_spawn_agents)
        self.spawn_goal_sub = rospy.Subscriber('spawn_goal_topic', String,self.debug_spawn_goals)
    

    def parse_string_to_map(self, map_string):
        self.map_str = str(map_string.data)
        rows = str(map_string.data).split(ROW_SEPARATOR)
        self.map_height = len(rows)
        for y_ind, row in enumerate(rows):
            columns = row.split(COL_SEPARATOR)
            self.map_width = len(columns)
            for x_ind, col in enumerate(columns):
                features = col.split('_')
                # agent found
                entity_type = features[FEATURE_ENTITY_IND]
                if EntityType(entity_type) == EntityType.AGENT:
                    agent_dir = Direction(int(features[FEATURE_DIR_IND]))
                    agent = Agent(y_ind, x_ind, agent_dir)
                    self.agent_dict[agent.id] = agent
                elif EntityType(entity_type) == EntityType.SHELF:
                    shelf = Shelf(y_ind, x_ind)
                    self.shelf_dict[shelf.id] = shelf
                elif EntityType(entity_type) == EntityType.GOAL:
                    goal = Goal(y_ind, x_ind)
                    self.goal_dict[goal.id] = goal
                else:
                    pass

    def map_string(self):
        map_str_arr = np.chararray((self.map_height, self.map_width), itemsize=10, unicode=True)
        map_str_arr[:] = '0'

        for goal_id in self.goal_dict.keys():
            goal = self.goal_dict[goal_id]
            map_str_arr[goal.y][goal.x] = 'G'

        for agent_id in self.agent_dict.keys():
            agent = self.agent_dict[agent_id]
            if map_str_arr[agent.y][agent.x][0] == 'G':
                map_str_arr[agent.y][agent.x] += '_A' + str(agent.cur_dir.value)
            else:
                map_str_arr[agent.y][agent.x] = 'A' + str(agent.cur_dir.value)

        for shelf_id in self.shelf_dict.keys():
            shelf = self.shelf_dict[shelf_id]
            if map_str_arr[shelf.y][shelf.x][0] == 'A' or map_str_arr[shelf.y][shelf.x][0] == 'G':
                map_str_arr[shelf.y][shelf.x] += "_S"
            else:
                map_str_arr[shelf.y][shelf.x] = 'S'

        new_map_arr = []
        for row in map_str_arr[:]:
            new_map_arr.append(COL_SEPARATOR.join(row.tolist()))

        new_map_str = "\n".join(new_map_arr)
        


        self.map_str = new_map_str + "\n" + "----------------"

        print('SENDING:\n ',self.map_str)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.update_map_pub.publish(self.map_str)
            rate.sleep()
            break

        return self.map_str

    def debug_spawn_goals(self, goals):
        goals = str(goals.data).split(',')
        for goal in goals:
            goal_attr = goal.split('_')
            is_slot_free = True
            for goal_id in self.goal_dict.keys():
                if self.goal_dict[goal_id].y == goal_attr[0] and self.goal_dict[goal_id].x == goal_attr[1]:
                    is_slot_free = False

            if is_slot_free:
                new_goal = Goal(int(goal_attr[0]),int(goal_attr[1]))
                self.goal_dict[new_goal.id] = new_goal

    def debug_spawn_agents(self, agents):
        agents = str(agents.data).split(',')
        for agent in agents:
            agent_attr = agent.split('_')
            is_slot_free = True
            for agent_id in self.agent_dict.keys():
                if self.agent_dict[agent_id].y == agent_attr[0] and self.agent_dict[agent_id].x == agent_attr[1]:
                    is_slot_free = False
            if is_slot_free:
                new_agent = Agent(int(agent_attr[0]), int(agent_attr[1]), Direction(int(agent_attr[2])))
                self.agent_dict[new_agent.id] = new_agent

    def debug_spawn_shelves(self, shelves):
        shelves = str(shelves.data).split(',')
        for shelf in shelves:
            shelf_attr = shelf.split('_')
            new_shelf = Shelf(int(shelf_attr[0]), int(shelf_attr[1]))
            self.shelf_dict[new_shelf.id] = new_shelf
    
    def debug_agents_actions(self, actions):
        actions = str(actions.data).split(',')
        for agent_actions in actions:
            agent_attr = agent_actions.split('_')
            agent = self.agent_dict[int(agent_attr[0])]
            des_action = Action(int(agent_attr[1]))

            ## if action is FORWARD then check collisions
            if des_action == Action.FORWARD:
                new_pos = self.simulate_move((agent.y,agent.x), agent.cur_dir, des_action)
                does_collide = self._does_collide(new_pos)
                if does_collide:
                    print('Invalid action ',des_action, 'for agent ', agent.id)
                    pass
                else:
                    self.agent_dict[agent.id].step(des_action)
            ##no neeed to check collision when turning
            elif des_action == Action.NONE or des_action == Action.RIGHT or des_action == Action.LEFT:
                self.agent_dict[agent.id].step(des_action)

            elif des_action == Action.LOAD:
                is_on_shelf, shelf_id = self._is_agent_on_shelf(agent)
                if is_on_shelf and  agent.carrying_shelf == None:
                    shelf = self.shelf_dict[shelf_id]
                    agent.load(shelf)
                    print('Agent ', agent.id, 'picked up the shelf ', shelf_id)
                else:
                    print('Invalid action ', des_action, 'for agent ', agent.id)
                    pass
            elif des_action == Action.UNLOAD:
                is_on_goal = self._is_agent_on_goal(agent)
                if is_on_goal and agent.carrying_shelf != None:
                    shelf = agent.carrying_shelf
                    agent.unload()
                    self.shelf_dict.pop(shelf.id)
                    print('Agent ', agent.id, 'left the shelf ', shelf.id)
                else:
                    print('Invalid action ', des_action, 'for agent ', agent.id)
                    pass
        self.map_string()

    def _init_agents_callback(self, msg):

        for i, (a, (x, y)) in enumerate(msg.data):
            cal_y, cal_x = self._con_to_disc(y, x)
            self.agent_dic[i] = Agent(cal_y, cal_x, a)
    def simulate_move(self, entity_pos : tuple, entity_dir : Direction, action : Action):
        new_y, new_x = entity_pos[0], entity_pos[1]
        if action == Action.FORWARD:
            if entity_dir == Direction.UP:
                new_y -= 1
            elif entity_dir == Direction.RIGHT:
                new_x += 1
            elif entity_dir == Direction.DOWN:
                new_y += 1
            else:
                new_x -= 1
        return (new_y, new_x)
    def _init_goals_callback(self, msg):
        for i, (x, y) in enumerate(msg.data):
            cal_y, cal_x = self._con_to_disc(y, x)
            self.goal_dic[i] = Goal(cal_x, cal_y)

    def _action_agents_callback(self, msg):

        for (agent, action) in enumerate(msg.data):
            # check collision
            action_type = Action(action)
            # pass by value
            lookup_agent = agent
            lookup_agent.step(action_type)
            is_collision = self._does_collide((lookup_agent.x, lookup_agent.y))
            if not is_collision:
                # check load
                if action_type == Action.LOAD:
                    # if agent is not carrying shelf and on a shelf
                    is_on_shelf, shelf_id = self._is_agent_on_shelf(agent)
                    if is_on_shelf and not agent.carrying_shelf:
                        # load shelf
                        agent.load(shelf_id)
                elif action_type == Action.UNLOAD:
                    is_on_goal = self._is_agent_on_goal(agent)
                    # if agent is on goal and carrying a shelf then can unload
                    if is_on_goal and agent.carrying_shelf:
                        self.shelf_dic.pop(agent.carrying_shelf_id)
                        agent.unload()
                        # TODO need to publish it ?
                elif action_type == Action.RIGHT or action_type == Action.LEFT:
                    agent.step(action_type)
                elif action_type == Action.FORWARD:
                    agent.step(action_type)
                    # if agent is carrying shelf then update also shelf s position
                    if agent.carrying_shelf:
                        shelf_id = agent.carrying_shelf_id
                        self.shelf_dic.get(shelf_id).x = agent.x
                        self.shelf_dic.get(shelf_id).y = agent.y
                elif action_type == Action.NONE:
                    agent.step(action_type)
            # TODO publish agent updated positions
            self.map_string()

    def _spawn_shelf_callback(self, msg):
        (ind, (x, y)) = msg.data
        if ind not in self.shelf_dic:
            self.shelf_dic[ind] = Shelf(y, x)

    def step(self):
        pass

    # for each simulation step

    # checks  if agent is on one of the goals
    def _is_agent_on_goal(self, agent: Agent) -> bool:

        for goal in self.goal_dict.values():
            if goal.x == agent.x and goal.y == agent.y:
                return True

        return False

    # checks  if agent is on one of the shelves
    def _is_agent_on_shelf(self, agent: Agent):

        for i, shelf in enumerate(self.shelf_dict.values()):
            if ((shelf.x - 1 == agent.x or shelf.x + 1 == agent.x) and (shelf.y == agent.y)) or ((shelf.y - 1 == agent.y or shelf.y + 1 == agent.y) and (shelf.x == agent.x)):
                return True, shelf.id

        return False, -1

    def _does_collide(self, first: Tuple):
        ## check agents
        for agent in self.agent_dict.values():
            if first[0] == agent.y and first[1] == agent.x:
                return True
        ## check shelves
        for shelf in self.shelf_dict.values():
            if first[0] == shelf.y and first[1] == shelf.x:
                return True
        ## check boundaries
        if first[0] < 0 or first[1] < 0 or first[0] >= self.map_height or first[1] >= self.map_width:
            return True
        return False

    # method to be used for converting self.map_image into array
    def _convert_image_into_array(self):
        pass

    # convert map into array and initialise self.map_array
    def _con_to_disc(self, map_y: float, map_x: float) -> Tuple:
        single_grid_width = self.map_width / self.grid_width
        single_grid_height = self.map_height / self.grid_height
        return (map_y % single_grid_height) + 1, (map_x % single_grid_width) + 1

    def _disc_to_con(self, grid_y: int, grid_x: int) -> Tuple:
        single_grid_width = self.map_width / self.grid_width
        single_grid_height = self.map_height / self.grid_height
        x_achse_offset = single_grid_width / 2
        y_achse_offset = single_grid_height / 2
        return (single_grid_height * grid_y) + y_achse_offset, (single_grid_width * grid_x) + x_achse_offset

    def on_shutdown(self):
        rospy.loginfo("agent warehouse shutting down.")


def run():
    rospy.init_node("warehouse", anonymous=False)
    print(
        "==================================\nagent warehouse node started\n=================================="
    )
    agents = [(Direction.RIGHT, (722, 300)), (Direction.RIGHT, (400, 200)), (Direction.DOWN, (134, 40)),
              (Direction.UP, (432, 90)), (Direction.UP, (190, 500))]
    goals = [(400, 800), (500, 800)]
    warehouse = AgentWarehouse()


    # rospy.on_shutdown(warehouse.on_shutdown)

    rospy.spin()


if __name__ == "__main__":
    run()