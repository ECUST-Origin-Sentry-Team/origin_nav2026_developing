import struct
import time
from referee_msg.msg import Referee
from rm_interfaces.msg import Gimbal
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import rclpy.duration
from math import pi
import threading
import glob
import serial




# 连接串口
ser=0
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


class RNode(Node):
    def __init__(self):
        super().__init__("receive_c_node")
        self.declare_parameter('offset',0.005)
        self.br = TransformBroadcaster(self)
        self.publish_gimbal = self.create_publisher(Gimbal,"gimbal_status",10)
        self.publish_referee = self.create_publisher(Referee,"Referee",10)
        self.publisher_timer = self.create_timer(0.0067,self.publish_message)
        self.declare_parameter('use_fake_referee', True)
        self.declare_parameter('self_color', 'red')

        self.use_fake_referee = self.get_parameter('use_fake_referee').get_parameter_value().bool_value
        self.color = 1 if( self.get_parameter('self_color').get_parameter_value().string_value == 'red') else 0 #0 打红 1打蓝
        if self.use_fake_referee == True:
            self.get_logger().warning("Using fake referee, no referee data will be published")
        self.gimbal_msg = Gimbal()

    def receive_from_c(self):
        while 1:
            head = ser.read()
            #print(head.hex())
            if head.hex() == 'aa':
                rx_buffer = []
                #print('receive_message')
                rx_buffer.append(head.hex())
                length = ser.read()
                rx_buffer.append(length.hex())
                while len(rx_buffer) < int.from_bytes(length, byteorder='big'):  # 至少包含帧头、帧长度和CRC校验位的长度
                    rx_buffer.append(ser.read().hex())
                # print(rx_buffer)
                try:
                    referee_data, result = self.parse_message(rx_buffer)
                    if result != None:
                        self.gimbal_msg.yaw = result[0]
                        self.gimbal_msg.roll  = result[1]
                        self.gimbal_msg.pitch= result[2]
                        self.gimbal_msg.mode= self.color 
                    if referee_data != None and self.use_fake_referee == False:
                        self.publish_referee.publish(referee_data)
                        # 发布消息
                except Exception as e:
                    print("Error:",str(e))

    def parse_message(self,message):
        # 检查帧头4
        if message[0] != 'aa':
            raise ValueError("Invalid header")
        # 解析帧长度
        length = message[1]
        if len(message) != int(length,16):
            raise ValueError("Invalid message length")
        # 检查CRC校验位
        # crc = crc8(message[3:-1])
        # if crc != message[-1]:
        #     raise ValueError("CRC check failed")
        # 解析命令字
        cmd_id = message[2]
        referee_result = []
        referee_data = None
        if cmd_id == '18':        
            referee_result.append(
                struct.unpack('<H',bytes.fromhex(''.join(message[3:5])))[0])#剩余血量 uint16
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[5:7]))        #总血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<B',
                    bytes.fromhex(''.join(message[7]))          #比赛阶段  uint8
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[8:10]))          #比赛阶段剩余时间  uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[10:12]))          #剩余经济 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[12:14]))          #剩余子弹 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[14:16]))          #红1血量 uint16
                                    )[0]
                )
            
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[16:18]))          #红2血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[18:20]))          #红3血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[20:22]))          #红4血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[22:24]))          #红7血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[24:26]))          #红前哨血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[26:28]))          #红基地血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[28:30]))          #蓝1血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[30:32]))          #蓝2血量 uint16
                                    )[0]
                )
            
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[32:34]))          #蓝3血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[34:36]))          #蓝4血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[36:38]))          #蓝7血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[38:40]))          #蓝前哨血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<H',
                    bytes.fromhex(''.join(message[40:42]))          #蓝基地血量 uint16
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<I',
                    bytes.fromhex(''.join(message[42:46]))          # rfid uint32
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<I',
                    bytes.fromhex(''.join(message[46:50]))          #增益点占领状态 uint32
                                    )[0]
                )
            referee_result.append(
                struct.unpack('<B',
                    bytes.fromhex(''.join(message[50]))          #受伤原因 uint8
                                    )[0]
                )
            
            referee_result.append(
                struct.unpack('<B',
                    bytes.fromhex(''.join(message[51]))          #敌方英雄位置 uint8
                                    )[0]
                )

            referee_result.append(
                struct.unpack('<?',
                    bytes.fromhex(''.join(message[52]))          #是否需要回到堡垒 bool
                                    )[0]
                )
            referee_data = Referee()
            referee_data.remain_hp = referee_result[0]
            referee_data.max_hp = referee_result[1]
            referee_data.game_progress = referee_result[2]
            referee_data.stage_remain_time = referee_result[3]
            referee_data.coin_remaining_num = referee_result[4]
            referee_data.bullet_remaining_num_17mm = referee_result[5]
            referee_data.red_1_hp = referee_result[6]
            referee_data.red_2_hp = referee_result[7]
            referee_data.red_3_hp = referee_result[8]
            referee_data.red_4_hp = referee_result[9]
            referee_data.red_7_hp = referee_result[10]
            referee_data.red_outpost_hp = referee_result[11]
            referee_data.red_base_hp = referee_result[12]
            referee_data.blue_1_hp = referee_result[13]
            referee_data.blue_2_hp = referee_result[14]
            referee_data.blue_3_hp = referee_result[15]
            referee_data.blue_4_hp = referee_result[16]
            referee_data.blue_7_hp = referee_result[17]
            referee_data.blue_outpost_hp = referee_result[18]
            referee_data.blue_base_hp = referee_result[19]
            referee_data.rfid_status = referee_result[20]
            referee_data.event_type = referee_result[21]
            referee_data.hurt_type = referee_result[22]
            referee_data.enemy_hero_pos = referee_result[23]
            referee_data.return_fortress = referee_result[24]



            # print(referee_data)
        result = None
        if cmd_id == '14':
            result = []
            #print("Invalid command ID")
            for i in range(3, len(message)-2, 4):
                string = ''.join(message[i:i + 4])  # 每四个十六进制数组合成一个字符串
                bytes_data = bytes.fromhex(string)
                float_value = struct.unpack('<f', bytes_data)[0]
                result.append(float_value)
            result.append(struct.unpack('<B', bytes.fromhex(message[-2]))[0])
        
            #print(result)
        return referee_data, result


    def publish_message(self):
        current_time=self.get_clock().now()
        offset=self.get_parameter('offset').get_parameter_value().double_value
        adjust_time=current_time+rclpy.duration.Duration(seconds=offset)
        t = TransformStamped()

        t.header.stamp = adjust_time.to_msg()
        t.header.frame_id = 'base_link_fake'
        t.child_frame_id = 'gimbal_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler((pi/180)*self.gimbal_msg.roll, (pi/180)*self.gimbal_msg.pitch, (pi/180)*self.gimbal_msg.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

        # self.get_logger().info(str(gimbal_msg))
        # print(t.transform.rotation)
        self.publish_gimbal.publish(self.gimbal_msg)

def main(args=None):
    rclpy.init(args=args)
    find_usb_devices()
    receive_c_node = RNode()
    thread = threading.Thread(target=receive_c_node.receive_from_c)
    thread.start()
    rclpy.spin(receive_c_node)
    receive_c_node.destroy_node()
    rclpy.shutdown()
