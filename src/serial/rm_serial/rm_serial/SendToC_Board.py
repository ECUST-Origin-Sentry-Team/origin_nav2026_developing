import struct
import serial
import time
import rclpy
import rclpy.duration
from rclpy.node import Node, Publisher
from rm_interfaces.msg import GimbalCmd
import rclpy.publisher
from std_msgs.msg import String,Bool,Float32,Int32
import tf_transformations
from math import pi
from geometry_msgs.msg import Twist
from referee_msg.msg import Referee
import numpy
import glob



# 定义帧头和命令字
HEADER = 0xAA
CMD_ID_AUTOAIM_DATA_RX= 0x81


ser=0
# 定义结构体
class All_Data_Rx:
    def __init__(self, yaw_aim, pitch_aim,fire_or_not,vx, vy, rotate, yaw_speed_nav, pitch_mode_nav,super_cap_mode,back,reach_goal):
        self.yaw_aim = yaw_aim
        self.pitch_aim= pitch_aim
        self.fire_or_not= fire_or_not
        self.vx = vx
        self.vy = vy
        self.rotate =rotate
        self.yaw_speed_nav = yaw_speed_nav
        self.pitch_mode_nav = pitch_mode_nav
        self.super_cap_mode = super_cap_mode
        self.back = back
        self.reach_goal =reach_goal

             
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
def pack_all(data):
    return struct.pack('<ff??fffffB?B', data.yaw_aim, \
                                        data.pitch_aim,\
                                        data.fire_or_not,\
                                        0.0,
                                        data.vx,\
                                        data.vy, \
                                        data.rotate, \
                                        data.yaw_speed_nav, \
                                        data.pitch_mode_nav,\
                                        data.super_cap_mode,\
                                        data.back,\
                                        data.reach_goal)




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
        self.sub_yaw = self.create_subscription(Float32,'nav_yaw',self.yaw_callback,10)
        self.sub_dip_angle = self.create_subscription(Float32,"/dip_angle",self.dip_angle_callback,10)

        self.sub_pitch = self.create_subscription(Bool,'nav_pitch',self.pitch_callback,10)
        self.sub_rot = self.create_subscription(Bool,"/skip_aim",self.inv_callback,10)
        self.sub_reach_goal = self.create_subscription(Bool,"/reach_hero",self.goal_callback,10)

        self.inv_dec = False
        # self.clock_timer = self.create_timer(6,self.clock_callback)

        self.rotate = 22000 #小陀螺逻辑由电控控制，导航只需要传一个定值
        self.pitch = 0
        self.vx = 0.0
        self.vy = 0.0
        self.v_yaw = 1.0

        self.yaw_aim = 0.0
        self.msg_fire = 0
        self.pitch_aim = 0.0
        self.super_cap_mode = 1
        self.back = False
        self.reach_goal = False

        self.clock = 1
        
    def pitch_callback(self,msg:Bool):
        if msg.data == True:
            self.pitch = 1
        else:
            self.pitch = 0

    def dip_angle_callback(self, msg:Float32):
        degree = msg.data
        if degree < 8.0:
            self.super_cap_mode = 0
        else:
            self.super_cap_mode = 2
    # def clock_callback(self):
    #     if self.clock == 1:
    #         self.clock = -1
    #     else:
    #         self.clock = 1

    def yaw_callback(self,msg:Float32):
        self.v_yaw = 1.0
        # if self.vx == 0.0 and self.vy == 0.0 and msg.data == 1.5: #msg.data是跑动时候的转速
        #     self.v_yaw = self.clock*msg.data*2.0
        # else:
        #     self.v_yaw = self.clock*msg.data

    def nav_callback(self, msg:Twist):
         self.vx=msg.linear.x
         self.vy=msg.linear.y

    def gimbal_callback(self,msg):
        self.msg_fire=msg.fire_advice
        self.yaw_aim=msg.yaw
        self.pitch_aim=msg.pitch

    def inv_callback(self,msg):
        self.inv_dec=msg.data
    def back_callback(self,msg):
        self.back=msg.data
    def goal_callback(self,msg):
        self.reach_goal=msg.data

    def send_all(self):
        data = All_Data_Rx(
            self.yaw_aim,
            self.pitch_aim,
            self.msg_fire,
            self.vx,
            self.vy,
            self.rotate,
            self.v_yaw,
            self.pitch,
            self.super_cap_mode,
            self.back,
            self.reach_goal
        )
        message = build_all_message(data)
        ser.write(message)
def main(args=None):
    rclpy.init(args=args)
    find_usb_devices()
    send_c_node = SNode()
    rclpy.spin(send_c_node)
    send_c_node.destroy_node()
    rclpy.shutdown()
