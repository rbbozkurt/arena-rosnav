#!/usr/bin/env python3

from enum import Enum

from re import A
from tabnanny import check
from typing import List, Tuple, Optional, Dict
import numpy as np
import rospy

from binascii import a2b_base64
from json import JSONDecodeError
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from std_msgs.msg import String

from sensor_msgs.msg import LaserScan
# viz
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
# arena 
import math
import numpy as np
from enum import Enum, IntEnum
import math

from scipy.interpolate import RegularGridInterpolator

ID_DECIMALS = 1

class Action(IntEnum):
    NOOP = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    HANDLE_LOAD = 4
    
class Direction(IntEnum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3


class EntityType(Enum):
    SHELF = "S"
    GOAL = "G"
    AGENT = "A"


class Goal:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y


class Shelf:
    counter = 0
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.id = Shelf.counter
        Shelf.counter += 1


#TODO WRITE ACTION PUBLISHER 

class Agent:
    counter = 0

    def __init__(self, dir, x,y):
        self.id = Agent.counter
        Agent.counter += 1
        self.x = x
        self.y = y
        self.pre_dir = int(dir) # todo save update pre dir
        self.dir = dir
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
        self.pre_dir = self.dir
        if action == Action.LEFT:
            self.cur_act = Action.NOOP
            if Direction(self.dir) == Direction.RIGHT:
                self.dir = int(Direction.UP)
            elif Direction(self.dir) == Direction.UP:
                self.dir = int(Direction.LEFT)
            elif Direction(self.dir) == Direction.LEFT:
                self.dir = int(Direction.DOWN)
            elif Direction(self.dir) == Direction.DOWN:
                self.dir = int(Direction.RIGHT)
        elif action == Action.RIGHT:
            self.cur_act = Action.NOOP
            if Direction(self.dir) == Direction.RIGHT:
                self.dir = int(Direction.DOWN)
            elif Direction(self.dir) == Direction.DOWN:
                self.dir = int(Direction.LEFT)
            elif Direction(self.dir) == Direction.LEFT:
                self.dir = int(Direction.UP)
            elif Direction(self.dir) == Direction.UP:
                self.dir = int(Direction.RIGHT)
        elif action == Action.NOOP:
            self.cur_act = action
        elif action == Action.FORWARD:
            self.cur_act = action
            if Direction(self.dir) == Direction.DOWN:
                self.y = self.y + 1
            elif Direction(self.dir) == Direction.UP:
                self.y = self.y - 1
            elif Direction(self.dir) == Direction.LEFT:
                self.x = self.x - 1
            elif Direction(self.dir) == Direction.RIGHT:
                self.x = self.x + 1

            if self.carrying_shelf:
                #todo update shelf position but shelf dict in warehouse ?!
                self.carrying_shelf_id.move(self.y, self.x)
            
                
        elif action == Action.HANDLE_LOAD:
            self.cur_act = Action.NOOP
            if self.carrying_shelf:
                # TODO DROP SHELF
                print()
            else:
                self.carrying_shelf = True
                # TODO PICKUP SHELF

def find_nearest(array, value):
    idx = (np.abs(np.asarray(array) - value)).argmin()
    print(idx)
    return array[idx]


class Warehouse:

    def init_agents(self,agents):
        i = 0
        for agent in agents:
            #self.agent_dict[i] = Agent(agent[0],agent[1][0],agent[1][1])
            i+= 1

    def __init__(self, width, height, map_width, map_height, agents, goal):

        self.grid_matrix = np.zeros((width,height))
        self.box_w = width/100
        self.box_h = height/100
        self.w = np.arange(0,self.box_w)
        self.h = np.arange(0,self.box_h)

        
        self.map_grid_height = 0
        self.map_grid_width = 0

        self.agent_list = []
        self.agent_dict = {}
        #self.init_agents(agents)

        self.shelf_list = []
        self.shelf_dict = {}
        self.goal = goal

        
        # subscriptions
        self.sub_agent_action = rospy.Subscriber("/agent_action_topic", Vector3, self.cb_agent_action )
        self.sub_agent_debug_action = rospy.Subscriber("/agent_action_debug", String, self.cb_debug_agent_action)
        #self.sub_goal = rospy.Subscriber("/goal_init_topic", Vector3, self.cb_sub_goal ) # todo??
        self.sub_shelves = rospy.Subscriber("/shelf_topic", Vector3, self.cb_shelves ) # spawning shelves

        # publishers
        self.agent_pos_pub = rospy.Publisher("/agent_position_topic", String, self.cb_execute_action,queue_size=10)
        self.agent_handle_load_pub = rospy.Publisher("/agent_handle_load_topic", String,self.cb_handle_load,queue_size=10)

        self.test_msg = rospy.Subscriber("map_topic",String, self.read_parse_map)
        
        self.update_map_pub = rospy.Publisher("update_map_topic",String,queue_size=10)
        self.spawn_shelf_sub = rospy.Subscriber('spawn_shelf_topic', String,self.cb_spawn_shelf)
        self.spawn_agent_sub = rospy.Subscriber('spawn_agent_topic', String,self.cb_spawn_agent)
    
        self.map_str = ""

        ##iterate over the map to find goal and the shelf
    def cb_debug_agent_action(self,data):
        actions = data.data.split(',')
        for agent_actions in actions:
            agent_attr = agent_actions.split('_')
            self.agent_dict[int(agent_attr[0])].step(Action(int(agent_attr[1])))
            self.update_map()

    def read_parse_map(self,data):
        
        self.map_str = str(data.data)

        rospy.loginfo(rospy.get_caller_id() + ' RECIEVED : %s', data.data)
        #print(data.data)    
        #for line in map_str:
        lines = self.map_str.split('/')
        
        self.map_grid_height = len(lines)
        self.map_grid_width = len(lines[0].split(','))

        rospy.loginfo('height %d width %d',self.map_grid_height, self.map_grid_width)


        for i in range(len(lines)):
            row = lines[i].split(',')
            for j in range (len(row)):
                if row[j] == '0':
                    continue
                
                elif row[j][0] == 'A': 
                    # todo find better way for shelf carrying agents
                    agent = Agent(int(row[j][1]),i,j)
                    self.agent_dict[agent.id] = agent
                    rospy.loginfo('AGENT FOUND AT : %d - %d , dir : %d', agent.x,agent.y,agent.dir)    
            
                    rospy.loginfo('AGENT ID : %d ', self.agent_dict[agent.id].id) 
                elif row[j][0] == 'S':

                    shelf = Shelf(i,j)
                    self.shelf_dict[shelf.id] = shelf
                    rospy.loginfo('SHELF FOUND AT : %d - %d', shelf.x,shelf.y)    
                    
                
                elif row[j][0] == 'G':
                    goal = Goal(i,j)
                    self.goal = goal
                    rospy.loginfo('GOAL FOUND AT : %d - %d', goal.x,goal.y)    
            
    
    def cb_spawn_agent(self,data):
        
        rospy.loginfo(rospy.get_caller_id() + ' RECIEVED : %s', data.data)
        a_str = str(data.data).split('_')
        
        new_agent  = Agent(int(a_str[0]),int(a_str[1]),int(a_str[2]))
        self.agent_dict[new_agent.id] = new_agent
        self.update_map()
        # TODO check if spawning point is valid

    def cb_spawn_shelf(self,data):
        
        rospy.loginfo(rospy.get_caller_id() + ' RECIEVED : %s', data.data)
        a_str = str(data.data).split('_')
        
        new_shelf  = Shelf(int(a_str[0]),int(a_str[1]))
        self.shelf_dict[new_shelf.id] = new_shelf
        self.update_map()
        
        # TODO check if spawning point is valid

    def update_map(self):        

        map_str_arr = np.chararray((self.map_grid_height, self.map_grid_width),itemsize=10,unicode=True)
        map_str_arr[:] = '0'

        map_str_arr[self.goal.x][self.goal.y] = 'G'

        for agent_id in self.agent_dict.keys():
            agent = self.agent_dict[agent_id]
            if map_str_arr[agent.x][agent.y][0] == 'G':
                map_str_arr[agent.x][agent.y] += '_A'+ str(int(agent.dir))
            else:
                map_str_arr[agent.x][agent.y] = 'A'+ str(int(agent.dir))

        for shelf_id in self.shelf_dict.keys():
            shelf = self.shelf_dict[shelf_id]
            if map_str_arr[shelf.x][shelf.y][0] == 'A' or map_str_arr[shelf.x][shelf.y][0] == 'G':
                map_str_arr[shelf.x][shelf.y] += "_S"
            else:
                map_str_arr[shelf.x][shelf.y] = 'S'

        new_map_arr = []
        for row in map_str_arr[:]:
            s = ','
            new_map_arr.append(s.join(row.tolist()))
        s = '/'
        new_map_str = s.join(new_map_arr)

        self.map_str = new_map_str
            
        rospy.loginfo(rospy.get_caller_id() + ' SENDING NEW MAP : %s', new_map_str)

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.update_map_pub.publish(new_map_str)
            break

    def assign_agent_to_shelf(self):
        # TODO f.e. manhattan distance to shelf
        pass

    def build_grid_graph(self):
        # TODO build graph with edges move,right,left
        pass
    
    def find_move(self):
        # TODO find what is the next move using the grid graph
        pass
    
    def execute_move(self):
        # TODO execute found move and update map()
        pass


    def calculate_action():
        # agent - shelf matching
        # agent 1 rotation, 0-2
        #monte-carlo / propagation /  tree search
        pass

    
    def cb_execute_action(): #AX_X1_Y1_DIR, AX_X1_Y1_DIR, 
        # call this inside loop and publish to task manager the new calc agent values
        pass
    
    
    def cb_handle_load(shelf_id): #in agent pickup shelf + AgentID-ShelfID
        # publish to task manager
        pass



    def map_to_grid(self,x,y):
        return math.floor(x/100), math.floor(y/100)

    def grid_to_map(self,x,y):
        return x*100, y*100 # todo add offset


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
    rospy.init_node("warehouse", anonymous=False)
    print('==================================\narena warehouse node started\n==================================')

    warehouse = Warehouse(800, 600, 8, 6, [], [])
    rospy.on_shutdown(warehouse.on_shutdown)

    rospy.spin()


if __name__ == "__main__":
    run()

'''
        for agent_id in self.agent_dict.keys():
            print(self.agent_dict[agent_id].id)
            agent_pos.append([self.agent_dict[agent_id].x,self.agent_dict[agent_id].y,self.agent_dict[agent_id].dir])

        shelf_pos = []
        for shelf_id in self.shelf_dict.keys():
            shelf_pos.append([self.shelf_dict[shelf_id].x,self.shelf_dict[shelf_id].y])

        lines = self.map_str.split('/')

        lines_new = []

        for i in range (len(lines)):
            line = lines[i].split(',')
            for j in range (len(line)):
                for agent in agent_pos:
                    if [i,j] == agent[0:2]:
                        line[j] = 'A' + str(agent[2])
                        agent_pos.remove(agent)
                        continue
                for shelf in shelf_pos:
                    if [i,j] in shelf_pos:
                        line[j] = 'S'
                        continue
            sep = ','
            lines_new.append(sep.join(line))
        
        s = '/'
        print(lines_new)
        new_map_str = s.join(lines_new)
        '''
        