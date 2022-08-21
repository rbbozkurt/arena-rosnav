from enum import Enum
from typing import List, Tuple, Optional, Dict
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





class Agent:
    counter = 0

    def __init__(self, _map:np.array()):
        Agent.counter += 1
        self.id = Agent.counter
        self.x = None
        self.y = None
        self.map = _map
        self.cur_dir = None
        self.cur_act: Optional[Action] = None
        self.carrying_shelf: bool = None
        self.canceled_action: bool = None
        self.has_delivered: bool = False

    def unload(self):
        if self.carrying_shelf:
            self.carrying_shelf = None
            self.has_delivered = True
        return

    def load(self):
        if self.carrying_shelf:
            return
        else:
            self.carrying_shelf = False
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
                self.prev_x = self.x
                self.prev_y = self.y
                self.y = self.y + 1
            elif self.cur_dir == Direction.UP:
                self.prev_x = self.x
                self.prev_y = self.y
                self.y = self.y - 1
            elif self.cur_dir == Direction.LEFT:
                self.prev_x = self.x
                self.prev_y = self.y
                self.x = self.x - 1
            elif self.cur_dir == Direction.RIGHT:
                self.prev_x = self.x
                self.prev_y = self.y
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


class Warehouse:
    _N_AGENT_LAYER = 0
    _N_SHELF_LAYER = 1

    def __init__(self, map_image):
        self.map_image = map_image
        self.map_array = None
        self.agent_dic = {}
        self.shelf_dic = {}
        self.goal_dic = {}

    # method to be used for converting self.map_image into array
    def _convert_image_into_array(self):

    # convert map into array and initialise self.map_array

    def step(self):

    # for each simulation step

    # checks  if agent is on one of the goals
    def _is_agent_on_goal(self, agent: Agent) -> bool:

        for goal in self.goal_dic.values():
            if goal.x == agent.x and goal.y == agent.y:
                return True

        return False

    # checks  if agent is on one of the shelves
    def _is_agent_on_shelf(self, agent: Agent) -> bool:

        for shelf in self.shelf_dic.values():
            if shelf.x == agent.x and shelf.y == agent.y:
                return True

        return False

    def _con_to_disc(self, y : float, x : float):

def run():

    rospy.init_node("arena_tb3", anonymous=False)
    print(
        "==================================\narena node started\n=================================="
    )

    nn_tb3 = NN_tb3()
    rospy.on_shutdown(nn_tb3.on_shutdown)

    rospy.spin()


if __name__ == "__main__":
    run()
