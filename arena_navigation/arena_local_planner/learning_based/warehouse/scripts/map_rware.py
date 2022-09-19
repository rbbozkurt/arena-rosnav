#!/usr/bin/env python
    # license removed for brevity,
import rospy
from std_msgs.msg import String
    
''' AXY = AGENT W NO X AND DIR Y, SX SHELF WITH NO X, OBSTACLE O, GOAL G '''
''' AX-SY x agent id Y shelf id - carrying shelf '''
class Gridworld:
    
    def __init__(self):
        self.map = "0,A_2,0,0,0,A_1,0,0/0,0,0,0,0,0,0,0/0,0,S,0,0,0,0,0/0,0,0,0,0,0,S,0/0,G,0,0,0,0,0,0"
        rospy.init_node('talker', anonymous=True)
        
        self.agent_pos_sub = rospy.Subscriber("/agent_position_topic", String, self.cb_update_map)
        self.agent_handle_load_sub = rospy.Subscriber("/agent_handle_load_topic", String)
        self.map_pub = rospy.Publisher('map_topic', String,queue_size=10)

        self.debug_agent_action_pub = rospy.Publisher("agent_action_topic", String, queue_size=10)
        self.spawn_shelf_pub = rospy.Publisher('spawn_shelf_topic', String,queue_size=10)
        self.spawn_agent_pub = rospy.Publisher('spawn_agent_topic', String,queue_size=10)
        self.spawn_goal_pub = rospy.Publisher('spawn_goal_topic', String,queue_size=10)
        
        self.update_map_sub = rospy.Subscriber("update_map_topic", String, self.cb_update_map)

    def send_map(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.map_pub.publish(self.map)
            break

    
    def send_actions(self,actions_str):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.debug_agent_action_pub.publish(actions_str)
            break

    def spawn_agent(self,agent_str): #D_x1_y1 
        #update map
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.spawn_agent_pub.publish(agent_str)
            break

    
    def spawn_shelf(self,shelf_str): #x1_y1 
        #update map
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.spawn_shelf_pub.publish(shelf_str)
            break
        
    def spawn_goal(self,goal_str): #x1_y1 
        #update map
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.spawn_goal_pub.publish(goal_str)
            break

    
    def cb_update_map(self, data):
        
        
        self.map = str(data.data)
        
        rospy.loginfo(rospy.get_caller_id() + ' UPDATED NEW MAP : %s', self.map)

        lines = str(data.data).split('/')

        for line in lines:
            indices = line.split(',')
            for ind in indices:
                print(' ', ind ,' ',end=" ")
                                        
            print()

        return
    def cb_handle_load(self):
        #TODO
        return
    def debug_agent_action(self, agent_moves):
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.debug_agent_action_pub.publish(agent_moves)
            break


if __name__ == '__main__':
    
    g = Gridworld()
    g.send_map()
    g.spawn_agent("2_2_1,2_3_2")
    g.spawn_shelf("4_7,4_3")
    g.spawn_goal('3_4')
    g.send_actions("1_1,2_3,3_2,4_1")
    g.send_actions("3_4")
    g.spawn_agent("1_4_2")
    g.spawn_shelf("2_4")
    g.send_actions("5_4")
    g.send_actions("5_1")
    g.send_actions("5_1")
    g.send_actions("5_5")


    

    rospy.spin()