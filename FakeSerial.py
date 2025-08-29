import struct
import numpy as np
import cv2
from PIL import Image

import sys, os
sys.path.append(os.path.dirname(__file__))

from CRC import Get_CRC8_Check_Sum, Get_CRC16_Check_Sum

class map_robot_data_t_py:
    def __init__(self, initData = None):
        self.hero_position_x = 0
        self.hero_position_y = 0
        self.engineer_position_x = 0
        self.engineer_position_y = 0
        self.infantry_3_position_x = 0
        self.infantry_3_position_y = 0
        self.infantry_4_position_x = 0
        self.infantry_4_position_y = 0
        self.infantry_5_position_x = 0
        self.infantry_5_position_y = 0
        self.sentry_position_x = 0
        self.sentry_position_y = 0
        if type(initData) == bytearray:
            self.__build_from_bytearray__(initData)

    def __build_from_bytearray__(self, data):
        data_list = [struct.unpack_from("<H", data[i*2:i*2+2])[0] for i in range(12)]
        #print(data_list)
        self.hero_position_x = data_list[0]
        self.hero_position_y = data_list[1]
        self.engineer_position_x = data_list[2]
        self.engineer_position_y = data_list[3]
        self.infantry_3_position_x = data_list[4]
        self.infantry_3_position_y = data_list[5]
        self.infantry_4_position_x = data_list[6]
        self.infantry_4_position_y = data_list[7]
        self.infantry_5_position_x = data_list[8]
        self.infantry_5_position_y = data_list[9]
        self.sentry_position_x = data_list[10]
        self.sentry_position_y = data_list[11]

    def print_infos(self, visualize = False, map_image = np.zeros((750, 1400, 3))):
        info_to_print = ""
        info_to_print += "模拟发送数据解析:(单位cm)"
        info_to_print += f"|英雄:({self.hero_position_x}, {self.hero_position_y})"
        info_to_print += f"|工程:({self.engineer_position_x}, {self.engineer_position_y})"
        info_to_print += f"|3号步兵:({self.infantry_3_position_x}, {self.infantry_3_position_y})"
        info_to_print += f"|4号步兵:({self.infantry_4_position_x}, {self.infantry_4_position_y})"
        info_to_print += f"|5号步兵:({self.infantry_5_position_x}, {self.infantry_5_position_y})"
        info_to_print += f"|哨兵:({self.sentry_position_x}, {self.sentry_position_y})"
        print(info_to_print)
        if visualize:
            if self.hero_position_x != 0 or self.hero_position_y != 0:
                visualize_position = (int(self.hero_position_x/100/28*1400), 750-int(self.hero_position_y/100/15*750))
                cv2.circle(map_image, visualize_position, 5, (0,255,0), -1)
                cv2.putText(map_image, f"Hero({self.hero_position_x},{self.hero_position_y})", (visualize_position[0]+10, visualize_position[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
            if self.engineer_position_x != 0 or self.engineer_position_y != 0:
                visualize_position = (int(self.engineer_position_x/100/28*1400), 750-int(self.engineer_position_y/100/15*750))
                cv2.circle(map_image, visualize_position, 5, (0,255,0), -1)
                cv2.putText(map_image, f"Engineer({self.engineer_position_x},{self.engineer_position_y})", (visualize_position[0]+10, visualize_position[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
            if self.infantry_3_position_x != 0 or self.infantry_3_position_y != 0:
                visualize_position = (int(self.infantry_3_position_x/100/28*1400), 750-int(self.infantry_3_position_y/100/15*750))
                cv2.circle(map_image, visualize_position, 5, (0,255,0), -1)
                cv2.putText(map_image, f"Infantry3({self.infantry_3_position_x},{self.infantry_3_position_y})", (visualize_position[0]+10, visualize_position[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
            if self.infantry_4_position_x != 0 or self.infantry_4_position_y != 0:
                visualize_position = (int(self.infantry_4_position_x/100/28*1400), 750-int(self.infantry_4_position_y/100/15*750))
                cv2.circle(map_image, visualize_position, 5, (0,255,0), -1)
                cv2.putText(map_image, f"Infantry4({self.infantry_4_position_x},{self.infantry_4_position_y})", (visualize_position[0]+10, visualize_position[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
            if self.infantry_5_position_x != 0 or self.infantry_5_position_y != 0:
                visualize_position = (int(self.infantry_5_position_x/100/28*1400), 750-int(self.infantry_5_position_y/100/15*750))
                cv2.circle(map_image, visualize_position, 5, (0,255,0), -1)
                cv2.putText(map_image, f"Infantry5({self.infantry_5_position_x},{self.infantry_5_position_y})", (visualize_position[0]+10, visualize_position[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
            if self.sentry_position_x != 0 or self.sentry_position_y != 0:
                visualize_position = (int(self.sentry_position_x/100/28*1400), 750-int(self.sentry_position_y/100/15*750))
                cv2.circle(map_image, visualize_position, 5, (0,255,0), -1)
                cv2.putText(map_image, f"Sentry({self.sentry_position_x},{self.sentry_position_y})", (visualize_position[0]+10, visualize_position[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))
            cv2.imshow("FakeSerialVisualize", map_image)
            cv2.waitKey(1)

class FakeSerial_Radar:
    def __init__(self):
        pil_map_image = Image.open(os.path.join(os.path.dirname(__file__), "rm2025map.png")).resize((1400, 750))
        self.map_image = cv2.cvtColor(np.array(pil_map_image), cv2.COLOR_RGB2BGR)

    def write(self, packet, print_info=False):
        frame_header = packet[0:5]
        SOF = struct.unpack_from("B", frame_header[0:1])[0]
        data_length = struct.unpack_from("<H", frame_header[1:3])[0]
        seq = struct.unpack_from("B", frame_header[3:4])[0]
        CRC8 = struct.unpack_from("B", frame_header[4:5])[0]
        cmd_id = struct.unpack_from("<H", packet[5:7])[0]
        data = packet[7:7+data_length]
        frame_tail = struct.unpack_from("<H", packet[7+data_length:9+data_length])[0]
        if print_info:
            print(packet)
            if SOF == 0xA5:
                print("SOF Pass")
            else:
                print("SOF Error")
            if data_length == 24:
                print("data_length Pass")
            else:
                print("data_length Error")
            print(f"seq: {seq}")
            if CRC8 == Get_CRC8_Check_Sum(frame_header, 4):
                print("CRC8 Pass")
            else:
                print("CRC8 Error")
            if cmd_id == 0x0305:
                print("cmd_id Pass")
            else:
                print("cmd_id Error")
            if frame_tail == Get_CRC16_Check_Sum(packet, 7+data_length):
                print("frame_tail Pass")
            else:
                print("frame_tail Error")
        map_robot_data_py = map_robot_data_t_py(data)
        map_robot_data_py.print_infos(visualize = True, map_image = self.map_image.copy())
    
    def read_all(self):
        return b''