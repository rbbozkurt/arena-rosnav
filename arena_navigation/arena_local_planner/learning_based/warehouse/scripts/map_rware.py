#!/usr/bin/env python
    # license removed for brevity,
import rospy
from std_msgs.msg import String
    
''' AXY = AGENT W NO X AND DIR Y, SX SHELF WITH NO X, OBSTACLE O, GOAL G '''
''' AX-SY x agent id Y shelf id - carrying shelf '''
class Gridworld:
    
    def __init__(self):
        self.map = "0,A2,0,0,0,A1,0,0/0,0,0,0,0,0,0,0/0,0,S,0,0,0,0,0/0,0,0,0,0,0,S,0/0,G,0,0,0,0,0,0"
        rospy.init_node('talker', anonymous=True)
        
        self.agent_pos_sub = rospy.Subscriber("/agent_position_topic", String, self.cb_update_map)
        self.agent_handle_load_sub = rospy.Subscriber("/agent_handle_load_topic", String)
        self.map_pub = rospy.Publisher('map_topic', String,queue_size=10)
        self.spawn_shelf_pub = rospy.Publisher('spawn_shelf_topic', String,queue_size=10)
        self.spawn_agent_pub = rospy.Publisher('spawn_agent_topic', String,queue_size=10)
        self.update_map_sub = rospy.Subscriber("update_map_topic", String, self.cb_update_map)

    def send_map(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            self.map_pub.publish(self.map)
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

    
    def cb_update_map(self, data):
        
        
        self.map = str(data.data)
        
        rospy.loginfo(rospy.get_caller_id() + ' UPDATED NEW MAP : %s', self.map)
        return
    def cb_handle_load(self):
        #TODO
        return

if __name__ == '__main__':
    
    g = Gridworld()
    g.send_map()
    g.spawn_agent('0_1_2')
    
    g.spawn_shelf('0_1')

    rospy.spin()