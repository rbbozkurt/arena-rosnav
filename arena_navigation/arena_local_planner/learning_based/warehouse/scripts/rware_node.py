#!/usr/bin/env python3

from enum import Enum
from typing import List, Tuple, Optional, Dict
from warehouse.msg import AgentInitMessage
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


class Goal:
    counter = 0

    def __init__(self, y: int, x: int):
        self.x = x
        self.y = y


class Shelf:
    counter = 0

    def __init__(self, y: int, x: int):
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
        self.cur_act: Optional[Action] = None
        self.carrying_shelf: bool = None
        self.carrying_shelf_id: int = None
        self.canceled_action: bool = None
        self.has_delivered: bool = False

    def unload(self):
        if self.carrying_shelf:
            self.carrying_shelf = None
            self.has_delivered = True
        return

    def load(self, shelf_id: int):
        if self.carrying_shelf:
            return
        else:
            self.carrying_shelf = True
            self.carrying_shelf_id = shelf_id
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
                self.carrying_shelf.move(self.y, self.x)
        elif action == Action.LOAD:
            self.cur_act = Action.NONE
            self.load()
            if self.carrying_shelf:
                self.carrying_shelf.move(self.y, self.x)
        elif action == Action.UNLOAD and self.carrying_shelf:
            self.cur_act = Action.NONE
            self.unload()


class AgentWarehouse:

    def __init__(self, _map_width: float, _map_height: float, _grid_width: int, _grid_height: int, _agents: list,
                 _goals: list):

        self.map_width = _map_width
        self.map_height = _map_height
        self.grid_width = _grid_width
        self.grid_height = _grid_height

        self.agent_dic = {}
        self.shelf_dic = {}
        self.goal_dic = {}

        self._debug_init_agents(_agents)
        self._debug_init_goals(_goals)
        # subscriptions
        self.agent_action_subs = rospy.Subscriber("/agent_action_topic", Action, )
        self.agent_init_subs = rospy.Subscriber("/agent_init_topic", Agent, )
        self.goal_init_subs = rospy.Subscriber("/goal_init_topic", Tuple, )
        self.shelf_spawn_subs = rospy.Subscriber("/shelf_topic", Tuple, )

        # publishers
        self.agent_pos_pub = rospy.Publisher("/agent_position_topic", Agent)
        self.agent_load_pub = rospy.Publisher("/agent_load_topic", Agent)
        self.agent_unload_pub = rospy.Publisher("/agent_unload_topic", Agent)

        ##iterate over the map to find goal and the shelf

    def _debug_init_goals(self, goals: list):
        for i, (x, y) in enumerate(goals):
            cal_y, cal_x = self._con_to_disc(y, x)
            self.goal_dic[i] = Goal(cal_x, cal_y)

    def _debug_init_agents(self, agents: list):
        for i, (a, (x, y)) in enumerate(agents):
            cal_y, cal_x = self._con_to_disc(y, x)
            self.agent_dic[i] = Agent(cal_y, cal_x, a)

    def _init_agents_callback(self, msg):

        for i, (a, (x, y)) in enumerate(msg.data):
            cal_y, cal_x = self._con_to_disc(y, x)
            self.agent_dic[i] = Agent(cal_y, cal_x, a)

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
            is_collision = self._does_collide((lookup_agent.x, lookup_agent.y));
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

    def _spawn_shelf_callback(self, msg):
        (ind, (x, y)) = msg.data
        if ind not in self.shelf_dic:
            self.shelf_dic[ind] = Shelf(y, x)

    def step(self):

    # for each simulation step

    # checks  if agent is on one of the goals
    def _is_agent_on_goal(self, agent: Agent) -> bool:

        for goal in self.goal_dic.values():
            if goal.x == agent.x and goal.y == agent.y:
                return True

        return False

    # checks  if agent is on one of the shelves
    def _is_agent_on_shelf(self, agent: Agent):

        for i, shelf in enumerate(self.shelf_dic.values()):
            if shelf.x == agent.x and shelf.y == agent.y:
                return True, i

        return False, -1

    def _does_collide(self, first: Tuple):
        for agent in self.agent_dic.values():
            if first[0] == agent.x and first[1] == agent.y:
                return True
        return False

    # method to be used for converting self.map_image into array
    def _convert_image_into_array(self):

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
    rospy.init_node("agent warehouse", anonymous=False)
    print(
        "==================================\nagent warehouse node started\n=================================="
    )
    agents = [(Direction.RIGHT, (722, 300)), (Direction.RIGHT, (400, 200)), (Direction.DOWN, (134, 40)),
              (Direction.UP, (432, 90)), (Direction.UP, (190, 500))]
    goals = [(400, 800), (500, 800)]
    warehouse = AgentWarehouse(800, 600, 8, 6, agents, goals)
    rospy.on_shutdown(warehouse.on_shutdown)

    rospy.spin()


if __name__ == "__main__":
    run()
