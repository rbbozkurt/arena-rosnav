#!/usr/bin/env python
    # license removed for brevity,
import rospy
from std_msgs.msg import String
    
''' AXY = AGENT W NO X AND DIR Y, SX SHELF WITH NO X, OBSTACLE O, GOAL G '''
''' / is new line '''

class Gridworld:
    
    def __init__(self):
        self.map = ["0,A00,0,0,0,A13,0,0/0,0,0,0,0,0,0,0/0,0,S0,0,0,0,0,0/0,0,0,0,0,0,S1,0/0,G,0,0,0,0,0,0"]
        rospy.init_node('talker', anonymous=True)
        
        self.agent_pos_sub = rospy.Subscriber("/agent_position_topic", String, self.cb_update_map)
        self.agent_handle_load_sub = rospy.Subscriber("/agent_handle_load_topic", String)
        self.map_pub = rospy.Publisher('map_topic', String,queue_size=10)
    

    def send_map(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            for line in self.map:
                print(line)
                self.map_pub.publish(line)
            break


    def cb_update_map(self):
        #TODO
        return
    def cb_handle_load(self):
        #TODO
        return

if __name__ == '__main__':
    
    g = Gridworld()
    g.send_map()
    
