#!/usr/bin/env python3

from enum import Enum
from re import A
from tabnanny import check
from typing import List, Tuple, Optional, Dict
import numpy as np


class Action(Enum):
    NOOP = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    HANDLE_LOAD = 4
    
class Direction(Enum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3


class EntityType(Enum):
    SHELF = "S"
    GOAL = "G"
    AGENT = "A"


class Goal:
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
        self.counter += 1
        self.id = Agent.counter
        self.x = x
        self.y = y
        self.cur_dir = dir
        self.cur_act: Optional[Action] = None
        self.carrying_shelf: bool = None
        self.carrying_shelf_id: int = None
        self.canceled_action: bool = None
        self.has_delivered: bool = False

    def handle_load(self, shelf_id):
        if self.carrying_shelf:    
            self.carrying_shelf = None
            self.has_delivered = True
        else:
            self.carrying_shelf = True
            self.carrying_shelf_id = shelf_id
            self.has_delivered = False
        return

    def step(self, action: Action):

        DIRS = [Direction.UP, Direction.RIGHT, Direction.DOWN, Direction.LEFT]
        if action != Action.NOOP:
            self.cur_act = Action.NOOP
            if self.cur_act == Action.LEFT:
                self.cur_dir = DIRS[DIRS.index(self.cur_dir.value -1) % 4]
            if self.cur_act == Action.RIGHT:
                self.cur_dir = DIRS[DIRS.index(self.cur_dir.value +1) % 4]
        
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

        elif action == Action.HANDLE_LOAD:
            self.cur_act = Action.HANDLE_LOAD
            self.handle_load(-1) # todo ask how to get shelf id from task manager, inside action? or warehouse? 
            if self.carrying_shelf:
                self.carrying_shelf.move(self.y, self.x)
        else:
            self.cur_act == Action.NOOP

def find_nearest(array, value):
    idx = (np.abs(np.asarray(array) - value)).argmin()
    return array[idx]


class Warehouse:

    def __init__(self, width, height, map_width, map_height, agents, goal):

        self.grid_matrix = np.zeros((width,height))
        self.box_w = width/map_width
        self.box_h = height/map_height
        self.w = np.arange(0,self.box_w)
        self.h = np.arange(0,self.box_h)

        self.agent_dict = {}
        self.shelf_dict = {}
        self.goal = goal

        # subscriptions
        self.sub_agent_action = rospy.Subscriber("/agent_action_topic", Action, self.cb_agent_action )
        self.sub_goal = rospy.Subscriber("/goal_init_topic", Tuple, self.cb_sub_goal ) # todo??
        self.sub_shelves = rospy.Subscriber("/shelf_topic", Tuple, self.cb_shelves ) # spawning shelves

        # publishers
        self.agent_pos_pub = rospy.Publisher("/agent_position_topic", Agent, self.cb_execute_action)
        self.agent_handle_load_pub = rospy.Publisher("/agent_handle_load_topic", self.cb_handle_load)

        ##iterate over the map to find goal and the shelf

    
    def map_to_grid(self,x,y):
        return find_nearest(self.box_w,x),find_nearest(self.box_h,y)

    def grid_to_map(self,x,y):
        return self.box_w*x, self.box_h*y


    def is_action_valid(self, agent_s, pos_x,pos_y):
        for agent_id, positions in self.agent_dic.iteritems():
            if agent_id != agent_s:
                if pos_x != positions[0] and pos_y != positions[0]:
                    return True

        return False


    def cb_agent_action(self,msg): # Assuming we recieve sth like this [ [action, [x,y]], next agent]
        agent_ids = msg.agent_ids
        agent_id = 0
        for agent in msg.actions:
            action = Action(agent.action) #todo iterating agent isnt from type agent todo fix
            if is_action_valid(agent_id,agent.pos[0],agent.pos[1]):
                agent.x, agent.y = self.map_to_grid(agent.x,agent.y)
                agent.step(action)
                if action == Action.FORWARD:
                    if agent.carrying_shelf:
                        self.shelf_dict[agent.carrying_shelf_id] = agent.pos # agents new position written to shelves pos
                elif action == Action.HANDLE_LOAD:
                    if self.check_goal(agent):
                        # drop shelve publish to task manager!! ask next, agent handle load probably already done in step()
                        self.shelf_dict.pop(agent.carrying_shelf_id)
                        cb_handle_load(agent.carrying_shelf_id) # todo fix bug in step carrying shelf already set to none, find correct id
                        # todo publish to task manager!


            # zwischen step : calc grid pos to big map pos again then publish it
            agent.x, agent.y = self.grid_to_map(agent.x,agent.y)

            # add here publish new agent pos, and shelve dropped at goal etc.

            agent_id += 1

    def cb_handle_load(shelf_id):
        # publish to task manager
        pass

    def cb_execute_action():
        # call this inside loop and publish to task manager the new calc agent values
        pass

    # todo spawn shelf coming from topic
    def cb_shelves(self,msg): # assuming [x,y] without id
        k = self.shelf_dict.keys()
        if k != None: 
            last_id = k[-1]
            self.shelf_dict[last_id+1] = Shelf(msg.x,msg.y)
        else:
            self.shelf_dict[0] = Shelf(msg.x,msg.y)


    def check_goal(self, agent) :
        if self.goal.x == agent.x and self.goal.y == agent.y:
            return True
        return False
    
    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down Node.")

def run():
    rospy.init_node(" warehouse", anonymous=False)
    print('==================================\narena warehouse node started\n==================================')

    agents = [(Direction.RIGHT, (722, 300)), (Direction.RIGHT, (400, 200)), (Direction.DOWN, (134, 40)),
              (Direction.UP, (432, 90)), (Direction.UP, (190, 500))]
    goals = [0,0]

    warehouse = Warehouse(10, 10, 8, 6, agents, goals)
    rospy.on_shutdown(warehouse.on_shutdown)

    rospy.spin()


if __name__ == "__main__":
    run()
