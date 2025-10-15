import rclpy
from rclpy.node import Node
from referee_msg.msg import Referee
from std_msgs.msg import String,Bool,Int16,Int8
from geometry_msgs.msg import PoseStamped
import time


class Panel_Publisher(Node):

    def __init__(self,node_name='control_panel_pub'):
        super().__init__(node_name)
        self.game_state_publisher_=self.create_publisher(Referee,'Referee',10)
        self.timer_=self.create_timer(1,self.timer_callback)

        
        self.game_state=Referee()
        self.game_state.game_progress=0
        self.game_state.stage_remain_time=420
        self.game_state.remain_hp=700
        self.game_state.bullet_remaining_num_17mm=400
        self.game_state.red_outpost_hp=1000
        self.game_state.blue_outpost_hp=1000
        self.game_state.red_base_hp=1000
        self.game_state.blue_base_hp=1000
        self.game_state.red_1_hp=400

        
    def set_game_state(self,game_progress,stage_remain_time,remain_hp,bullet_remaining_num_17mm,red_outpost_hp,blue_outpost_hp,red_base_hp,blue_base_hp):
        self.game_state.game_progress=game_progress
        self.game_state.stage_remain_time=stage_remain_time
        self.game_state.remain_hp=remain_hp
        self.game_state.bullet_remaining_num_17mm=bullet_remaining_num_17mm
        self.game_state.red_outpost_hp=red_outpost_hp
        self.game_state.blue_outpost_hp=blue_outpost_hp
        self.game_state.red_base_hp=red_base_hp
        self.game_state.blue_base_hp=blue_base_hp

        
    def timer_callback(self):
        if(self.game_state.stage_remain_time>0):
            self.game_state.stage_remain_time=self.game_state.stage_remain_time-1
        else:
            self.game_state.game_progress=5
        
        