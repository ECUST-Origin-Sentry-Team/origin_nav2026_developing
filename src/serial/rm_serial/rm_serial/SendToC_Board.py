import struct
import serial
import time
import rclpy
from rclpy.node import Node
from rm_interfaces.msg import GimbalCmd
from rm_interfaces.msg import Gimbal
from std_msgs.msg import Bool,Float32
from rm_interfaces.msg import GimbalRegionCmd
from geometry_msgs.msg import Twist
import glob



# 定义帧头和命令字
HEADER = 0xAA
CMD_ID_AUTOAIM_DATA_RX= 0x81


ser=0

             
# CRC-8 校验表
CRC08_Table = [
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
]
# 计算CRC校验位
def crc8(data):
    crc = 0xff
    for byte in data:
        crc = CRC08_Table[crc ^ byte]
    return crc

# 连接串口
def find_usb_devices():
    global ser
    usb_devices = glob.glob('/dev/ttyACM*')
    for device in usb_devices:
        ser = serial.Serial(device, 921600)
        if ser.is_open:
            #print("Trying to connect to:", device)
            time.sleep(0.1)
            while ser.in_waiting>0:
                data = ser.read()
                if data:
                    print('device found:%s',ser)
                    return 1
            ser.close()
            continue
    print("No device found")
    return 0



# 打包结构体为字节流
def pack_all(field_values):
    format_string = '<' # 小端字节序
    values = []
    
    for field, fmt in field_values:
        format_string += fmt
        values.append(field)
    
    return struct.pack(format_string, *values)

# 构建消息
def build_all_message(data):
    data_bytes = pack_all(data)
    length = len(data_bytes) + 4  # 包含帧头、帧长度、命令字和校验位
    #print(length)
    crc = crc8(struct.pack('<BBB', HEADER, length, CMD_ID_AUTOAIM_DATA_RX) + data_bytes)
    return struct.pack('<BBB', HEADER, length, CMD_ID_AUTOAIM_DATA_RX) + data_bytes + struct.pack('<B', crc)



class SNode(Node):
    def __init__(self):
        super().__init__("send_c_node")
        self.subscription = self.create_subscription(GimbalCmd, 'serial/process_gimbal_aft_inv', self.gimbal_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)  # CHANGE
        self.publish_gimbal = self.create_subscription(Bool,"back",self.back_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)  # BACK
        self.publisher_timer = self.create_timer(0.0067,self.send_all)

        #导航
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel_chassis', self.nav_callback,rclpy.qos.qos_profile_sensor_data)
        self.sub_dip_angle = self.create_subscription(Float32,"/dip_angle",self.dip_angle_callback,10)
        self.sub_nav = self.create_subscription(GimbalRegionCmd, '/serial/gimbal_region_cmd', self.chassis_mode_callback,10)
        
        self.sub_pitch = self.create_subscription(Bool,'/serial/nav_pitch',self.pitch_callback,10)
        self.sub_reach_goal = self.create_subscription(Bool,"/reach_hero",self.goal_callback,10)


        self.publish_gimbal = self.create_subscription(Gimbal,"gimbal_status",self.yaw_C_board_callback,10)

        #----------------------辅助变量-------------------------------#
        self.yaw_C_board = 0.0              # 从C板传来的距离零点的距离 
        self.exp_gimbal_angle = 0.0         # 当前时刻期望的加上云台零点的角度
        self.in_region = False              # 判断是否为刚进入特殊路段



        #-----------------------模式相关数据---------------------------#
        self.chassis_mode  = 2              # 底盘模式(1跟头 2陀螺)
        self.pass_bumpy_mode = 0             # 是否正在gimbal_region_callback过颠簸路段
        self.pitch = 0                      # pitch为打人模式还是前哨模式
        #-----------------------自瞄相关数据---------------------------#
        self.yaw_aim = 0.0                  # 自瞄yaw角
        self.pitch_aim = 0.0                # 自瞄pitch角
        self.msg_fire = 0                   # 自瞄是否开火
        self.turn_back = False              # 进入大回环
        #-----------------------导航相关数据---------------------------#
        self.vx = 0.0                       # 导航速度x
        self.vy = 0.0                       # 导航速度x
        self.yaw_angle_follow_head = 0.0    # 当为跟头模式时云台预期角度
        self.super_cap_mode = 0             # 是否正在上坡
        self.reach_hero = False             # 是否到达抓英雄点位



    def pitch_callback(self,msg:Bool):
        if msg.data == True:
            self.pitch = 1
        else:
            self.pitch = 0
    def yaw_C_board_callback(self,msg:Gimbal):
        self.yaw_C_board = msg.yaw

    def chassis_mode_callback(self,msg:GimbalRegionCmd):
        self.chassis_mode = msg.chassis_mode
        self.pass_bumpy_mode = msg.pass_special_region
        self.yaw_angle_follow_head =msg.pass_region_angle
        if self.chassis_mode == 2 and self.pass_bumpy_mode == 0 : # 离开颠簸路段
            self.in_region = False
            self.exp_gimbal_angle = 0.0
        elif self.chassis_mode == 1 and self.pass_bumpy_mode == 1 and not self.in_region:
            self.in_region = True
            self.exp_gimbal_angle = msg.pass_region_angle + self.yaw_C_board
            print(self.exp_gimbal_angle)
        

    def dip_angle_callback(self, msg:Float32):
        degree = msg.data
        if degree < 8.0:
            self.super_cap_mode = 0
        else:
            self.super_cap_mode = 2
            
    def nav_callback(self, msg:Twist):
         self.vx=msg.linear.x
         self.vy=msg.linear.y

    def gimbal_callback(self,msg):
        self.msg_fire=msg.fire_advice
        self.yaw_aim=msg.yaw
        self.pitch_aim=msg.pitch
    def back_callback(self,msg):
        self.turn_back=msg.data
    def goal_callback(self,msg):
        self.reach_hero=msg.data

    '''
    pack format:
    Format  Python
    c       char                  
    ?       bool
    h       short               (-32768-32767)
    H       unsigned short      (0-65535)
    B       unsigned char       (0-255)
    i       int                 (-2147483648-2147483647)
    I       unsigned int        (0-4294967295)
    f       float
    d       double
    s       char[]
    '''
    def send_all(self):
        field_values = [
            
            (self.chassis_mode,'B'),
            (self.pass_bumpy_mode,'B'),
            (self.pitch, 'B'),

            (self.yaw_aim, 'f'),
            (self.pitch_aim, 'f'),
            (self.msg_fire, '?'),
            (self.turn_back, '?'),

            (self.vx, 'f'),
            (self.vy, 'f'),
            (self.exp_gimbal_angle,'f'),
            (self.super_cap_mode, 'B'),
            (self.reach_hero, 'B'),
        ]
        message = build_all_message(field_values)
        # print(message)
        ser.write(message)
def main(args=None):
    rclpy.init(args=args)
    find_usb_devices()
    send_c_node = SNode()
    rclpy.spin(send_c_node)
    send_c_node.destroy_node()
    rclpy.shutdown()
