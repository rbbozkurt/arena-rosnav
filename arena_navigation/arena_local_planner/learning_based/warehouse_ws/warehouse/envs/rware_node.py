#!/usr/bin/env python3
import string
import random
from enum import Enum, IntEnum
from typing import Tuple, Optional

import numpy as np

DEBUG = False
# debug true -> agent id A2(id), debug false -> agent orientation A2 (dir)

ROW_SEPARATOR = '/'
COL_SEPARATOR = ','
FEATURE_SEPARATOR = '_'
FEATURE_ENTITY_IND = 0
FEATURE_ID_IND = 2
FEATURE_DIR_IND = 1

class Action(Enum):
    NONE = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    LOAD = 4
    UNLOAD = 5


# in clockwise
class Direction(IntEnum):
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




class Goal:
    counter = 0

    def __init__(self, y: int, x: int):
        Goal.counter += 1
        self.id = Goal.counter
        self.id = 0
        self.x = x
        self.y = y


class Shelf:
    counter = 0

    def __init__(self, y: int, x: int):
        Shelf.counter += 1
        self.id = Shelf.counter
        self.id = 0
        self.x = x
        self.y = y


class Agent:
    counter = 0

    def __init__(self, y: int, x: int, dir: Direction):
        Agent.counter += 1
        self.id = Agent.counter
        self.id = 0
        self.x = x
        self.y = y
        self.cur_dir = dir
        self.carrying_shelf: Shelf = None
        self.carrying_shelf_id: int = None
        self.has_delivered: bool = False
        self.goal = None
        self.des_action_on_goal = None
        self.score = 0
        self.min_dis = 999

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
        self.free_shelves = {}
        self.goal_dict = {}
        self.map_str = None


        # subscriptions
        #
        # self.agent_action_subs = rospy.Subscriber("/agent_action_topic", Action, )
        # self.agent_init_subs = rospy.Subscriber("/agent_init_topic", Agent, )
        # self.goal_init_subs = rospy.Subscriber("/goal_init_topic", Tuple, )
        # self.shelf_spawn_subs = rospy.Subscriber("/shelf_topic", Tuple, )
        #

        # publishers
        # self.agent_pos_pub = rospy.Publisher("/agent_position_topic", Agent)
        # self.agent_load_pub = rospy.Publisher("/agent_load_topic", Agent)
        # self.agent_unload_pub = rospy.Publisher("/agent_unload_topic", Agent)

        ##iterate over the map to find goal and the shelf

    def parse_string_to_map(self, map_string: string):
        self.map_str = map_string
        rows = map_string.split(ROW_SEPARATOR)
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
                    self.free_shelves[shelf.id] = shelf
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
                if DEBUG:
                    map_str_arr[agent.y][agent.x] += '_A' + str(agent.id)
                else:
                    map_str_arr[agent.y][agent.x] += '_A' + str(agent.cur_dir.value)
            else:
                if DEBUG:
                    map_str_arr[agent.y][agent.x] = 'A' + str(agent.id)
                else:
                    map_str_arr[agent.y][agent.x] = 'A' + str(agent.cur_dir.value)

        for shelf_id in self.shelf_dict.keys():
            shelf = self.shelf_dict[shelf_id]
            if map_str_arr[shelf.y][shelf.x][0] == 'A' or map_str_arr[shelf.y][shelf.x][0] == 'G':
                map_str_arr[shelf.y][shelf.x] += "_S"
                if DEBUG:
                    map_str_arr[shelf.y][shelf.x] += str(shelf.id)

            else:
                map_str_arr[shelf.y][shelf.x] = 'S'
                if DEBUG:
                    map_str_arr[shelf.y][shelf.x] += str(shelf.id)

        new_map_arr = []
        for row in map_str_arr[:]:
            new_map_arr.append(COL_SEPARATOR.join(row.tolist()))

        new_map_str = "\n".join(new_map_arr)

        self.map_str = new_map_str + "\n" + "----------------"
        return self.map_str

    def debug_spawn_goals(self, goals: string):
        goals = goals.split(',')
        for goal in goals:
            goal_attr = goal.split('_')
            is_slot_free = True
            for goal_id in self.goal_dict.keys():
                if self.goal_dict[goal_id].y == goal_attr[0] and self.goal_dict[goal_id].x == goal_attr[1]:
                    is_slot_free = False

            if is_slot_free:
                new_goal = Goal(int(goal_attr[0]),int(goal_attr[1]))
                self.goal_dict[new_goal.id] = new_goal

    def debug_spawn_agents(self, agents: string):
        agents = agents.split(',')
        for agent in agents:
            agent_attr = agent.split('_')
            is_slot_free = True
            for agent_id in self.agent_dict.keys():
                if self.agent_dict[agent_id].y == agent_attr[0] and self.agent_dict[agent_id].x == agent_attr[1]:
                    is_slot_free = False
            if is_slot_free:
                new_agent = Agent(int(agent_attr[0]), int(agent_attr[1]), Direction(int(agent_attr[2])))
                self.agent_dict[new_agent.id] = new_agent

    def debug_spawn_shelves(self, shelves : string):
        shelves = shelves.split(',')
        for shelf in shelves:
            shelf_attr = shelf.split('_')
            new_shelf = Shelf(int(shelf_attr[0]), int(shelf_attr[1]))
            self.shelf_dict[new_shelf.id] = new_shelf
            self.free_shelves[new_shelf.id] = new_shelf
    
    def debug_agents_actions(self, actions : string):
        #actions = actions.split(',')
        s = '0_' + str(actions)
        d_actions = [s]
        for agent_actions in d_actions:
            agent_attr = agent_actions.split('_')
            agent = self.agent_dict[int(agent_attr[0])]
            des_action = Action(int(agent_attr[1]))

            ## if action is FORWARD then check collisions
            if des_action == Action.FORWARD:
                new_pos = self.simulate_move((agent.y,agent.x), agent.cur_dir, des_action)
                does_collide = self._does_collide(new_pos)
                if does_collide:
                    agent.score -= 0.5
                   # print('Invalid action ',des_action, 'for agent ', agent.id)
                    pass
                else:
                    if agent.carrying_shelf == None:
                        min = self.calc_min_dis(agent)
                        if min < agent.min_dis:
                            agent.min_dis = min
                            agent.score += 0.5
                    self.agent_dict[agent.id].step(des_action)
            ##no neeed to check collision when turning
            elif des_action == Action.NONE or des_action == Action.RIGHT or des_action == Action.LEFT:
                self.agent_dict[agent.id].step(des_action)

            elif des_action == Action.LOAD:
                is_on_shelf, shelf_id = self._is_agent_on_shelf(agent)
                if is_on_shelf and  agent.carrying_shelf == None:
                    shelf = self.shelf_dict[shelf_id]
                    self.free_shelves.pop(shelf.id)
                    agent.load(shelf)
                    agent.score += 2
                    agent.min_dis = 0
                    print('Agent ', agent.id, 'picked up the shelf ', shelf.id)
                else:
                    #print('Invalid action ', des_action, 'for agent ', agent.id)
                    pass
            elif des_action == Action.UNLOAD:
                is_on_goal = self._is_agent_on_goal(agent)
                if is_on_goal and agent.carrying_shelf != None:
                    shelf = agent.carrying_shelf
                    agent.unload()
                    agent.score += 1
                    self.shelf_dict.pop(shelf.id)
                    agent.min_dis = 999
                    print('Agent ', agent.id, 'left the shelf ', shelf.id)
                else:
                   # print('Invalid action ', des_action, 'for agent ', agent.id)
                    pass

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
    def calc_min_dis(self, agent : Agent):
        min = 999
        for shelf in self.free_shelves.values():
            dis = abs(agent.x - shelf.x) + abs(agent.y - shelf.y)
            if dis < min:
                min = dis
        return min

    def evaluate(self):

        if self.agent_dict[0].carrying_shelf == None and self.free_shelves :
            reward = -100 + abs(self.agent_dict[0].x - self.free_shelves[0].x) + abs(self.agent_dict[0].y - self.free_shelves[0].y)
        elif self.agent_dict[0].carrying_shelf and self._is_agent_on_goal(self.agent_dict[0]) :
            reward =  10000
        else:
            reward = -50 + abs(self.agent_dict[0].x - self.goal_dict[0].x) + abs(self.agent_dict[0].y - self.goal_dict[0].y)

        return reward

    def observe(self):
        #man_dist = [0]*len(self.agent_dict)
        #for i, agent in enumerate(self.agent_dict.keys()):
        #    if self.agent_dict[agent].
        dist = [0]
        if self.agent_dict[0].carrying_shelf:
            dist[0] = abs(self.goal_dict[0].x - self.agent_dict[0].x) + abs(self.goal_dict[0].y - self.agent_dict[0].y)
        elif not self.is_done() and self.free_shelves:
            dist[0] = abs(self.free_shelves[0].x - self.agent_dict[0].x) + abs(self.free_shelves[0].y - self.agent_dict[0].y)
        elif self.is_done():
            return tuple([0])
        return tuple(dist)

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
                for agent in self.agent_dict.values():
                    if agent.carrying_shelf != None and agent.carrying_shelf.id == shelf.id:
                        return False, -1
                return True, shelf.id

        return False, -1

    def is_done(self):
        if not self.shelf_dict and not self.free_shelves:
            print('!!!!!!!!!!!!!!!!!!!!!!!DONEEEEEEEEEEEEEEEE')
            return True
        else:
            return False

    def view(self):      

        map_str_arr = np.chararray((self.map_height, self.map_width),itemsize=10,unicode=True)
        map_str_arr[:] = '0'
        print(self.agent_dict[0].id)
        map_str_arr[self.goal_dict[0].y][self.goal_dict[0].x] = 'G'

        for agent_id in self.agent_dict.keys():
            agent = self.agent_dict[agent_id]
            if map_str_arr[agent.y][agent.x][0] == 'G':
                map_str_arr[agent.y][agent.x] += '_A'+ str(int(agent.cur_dir))
            else:
                map_str_arr[agent.y][agent.x] = 'A'+ str(int(agent.cur_dir))

        for shelf_id in self.shelf_dict.keys():
            shelf = self.shelf_dict[shelf_id]
            if map_str_arr[shelf.y][shelf.x][0] == 'A' or map_str_arr[shelf.y][shelf.x][0] == 'G':
                map_str_arr[shelf.y][shelf.x] += "_S"
            else:
                map_str_arr[shelf.y][shelf.x] = 'S'

        new_map_arr = []
        for row in map_str_arr[:]:
            s = ','
            new_map_arr.append(s.join(row.tolist()))
        s = '/'
        new_map_str = s.join(new_map_arr)

        self.map_str = new_map_str


        lines = self.map_str.split('/')

        for line in lines:
            line = line.split(',')
            for c in line:
                print('-' , c , '-', end='')
            print()
        
        
        print("_____________________________________________")

        

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
    # rospy.init_node("agent warehouse", anonymous=False)
    print(
        "==================================\nagent warehouse node started\n=================================="
    )

    warehouse = AgentWarehouse()
    warehouse.parse_string_to_map("0,A_2,0,0,0,A_1,0,0/0,0,0,0,0,0,0,0/0,0,S,0,0,0,0,0/0,0,0,0,0,0,S,0/0,G,0,0,0,0,0,0")

    warehouse.debug_spawn_agents("2_2_1,2_3_2")
    warehouse.debug_spawn_shelves("4_7,4_3")
    warehouse.debug_spawn_goals('3_4')
    warehouse.debug_spawn_agents("1_4_2")
    warehouse.debug_spawn_shelves("2_4")
    step_count = 0
    while len(warehouse.shelf_dict.values()):
        print(len(warehouse.shelf_dict.values()), " shelves left")
        actions = []
        agent_action = []
        #for i in range(1,len(warehouse.agent_dict.keys())+1):
        for i in range(1,2):
        
            action = random.randint(0, 5)
            actions.append(str(i)+'_'+str(action))
        warehouse.debug_agents_actions(",".join(actions))
        print(warehouse.map_string())
        step_count += 1
    for agent in warehouse.agent_dict.values():
        print(agent.score)
    print('step number : ', step_count)
    # rospy.on_shutdown(warehouse.on_shutdown)

    #rospy.spin()


#if __name__ == "__main__":
#    run()
