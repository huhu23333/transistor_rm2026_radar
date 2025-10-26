import struct
import numpy as np
import cv2
from PIL import Image
import queue

import sys, os
sys.path.append(os.path.dirname(__file__))

from CRC import Get_CRC8_Check_Sum, Get_CRC16_Check_Sum

class map_robot_data_t_py:
    def __init__(self, initData, fakeSerialVisualize_frame_queue):
        self.fakeSerialVisualize_frame_queue = fakeSerialVisualize_frame_queue
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
            self.__build_from_bytearray(initData)

    def __build_from_bytearray(self, data):
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

    def print_infos(self, print_info = True, visualize = True, map_image = np.zeros((750, 1400, 3)), real_serial = None):
        if print_info:
            info_to_print = "--------------------------------\n"
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
            map_image = cv2.resize(map_image, (700, 375))
            cv2.putText(map_image, f"real_serial : {real_serial.port if real_serial else 'None'}", (2,28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0))
            #cv2.imshow(f"FakeSerialVisualize | real_serial : {real_serial.port if real_serial else "None"}", map_image)
            self.fakeSerialVisualize_frame_queue.put(map_image)
            #cv2.waitKey(1) 

class FakeSerial_Radar:
    def __init__(self, print_info_TX = True, print_info_RX = True, visualize = True, real_serial = None, fakeSerialVisualize_frame_queue=queue.Queue(maxsize=0)):
        self.fakeSerialVisualize_frame_queue = fakeSerialVisualize_frame_queue
        pil_map_image = Image.open(os.path.join(os.path.dirname(__file__), "rm2025map.png")).resize((1400, 750))
        self.map_image = cv2.cvtColor(np.array(pil_map_image), cv2.COLOR_RGB2BGR)
        self.print_info_TX = print_info_TX
        self.print_info_RX = print_info_RX
        self.visualize = visualize
        self.real_serial = real_serial
        self.real_serial_data_temp = b""
        self.real_serial_data_temp_max_len = 1024

    def write(self, packet):
        if self.real_serial:
            self.real_serial.write(packet)
        frame_header = packet[0:5]
        SOF = struct.unpack_from("B", frame_header[0:1])[0]
        data_length = struct.unpack_from("<H", frame_header[1:3])[0]
        seq = struct.unpack_from("B", frame_header[3:4])[0]
        CRC8 = struct.unpack_from("B", frame_header[4:5])[0]
        cmd_id = struct.unpack_from("<H", packet[5:7])[0]
        data = packet[7:7+data_length]
        frame_tail = struct.unpack_from("<H", packet[7+data_length:9+data_length])[0]
        if self.print_info_TX:
            info_to_print = "--------------------------------\n"
            info_to_print += (f"TX packet: {packet}\n")
            if SOF == 0xA5:
                info_to_print += ("TX SOF Pass\n")
            else:
                info_to_print += ("TX SOF Error\n")
            if data_length == 24:
                info_to_print += ("TX data_length Pass\n")
            else:
                info_to_print += ("TX data_length Error\n")
            info_to_print += (f"TX seq: {seq}\n")
            if CRC8 == Get_CRC8_Check_Sum(frame_header, 4):
                info_to_print += ("TX CRC8 Pass\n")
            else:
                info_to_print += ("TX CRC8 Error\n")
            if cmd_id == 0x0305:
                info_to_print += ("TX cmd_id Pass\n")
            else:
                info_to_print += ("TX cmd_id Error\n")
            if frame_tail == Get_CRC16_Check_Sum(packet, 7+data_length):
                info_to_print += ("TX frame_tail Pass")
            else:
                info_to_print += ("TX frame_tail Error")
            print(info_to_print)
        map_robot_data_py = map_robot_data_t_py(data, self.fakeSerialVisualize_frame_queue)
        map_robot_data_py.print_infos(print_info = self.print_info_TX, visualize = self.visualize, map_image = self.map_image.copy(), real_serial = self.real_serial)
    
    def read_all(self):
        if self.real_serial:
            real_serial_data = self.real_serial.read_all()
            self.real_serial_data_temp += real_serial_data
            if len(self.real_serial_data_temp) > self.real_serial_data_temp_max_len:
                self.real_serial_data_temp = self.real_serial_data_temp[len(self.real_serial_data_temp)-self.real_serial_data_temp_max_len:]
            has_SOF_flag = True
            while has_SOF_flag:
                has_SOF_flag = False
                for SOF_index in range(len(self.real_serial_data_temp)):
                    if self.real_serial_data_temp[SOF_index] == 0xA5:
                        self.real_serial_data_temp = self.real_serial_data_temp[SOF_index:]
                        packet = self.real_serial_data_temp
                        if len(packet) <= 9:
                            break
                        frame_header = packet[0:5]
                        SOF = struct.unpack_from("B", frame_header[0:1])[0]
                        data_length = struct.unpack_from("<H", frame_header[1:3])[0]
                        if len(packet) < 9+data_length:
                            break
                        seq = struct.unpack_from("B", frame_header[3:4])[0]
                        CRC8 = struct.unpack_from("B", frame_header[4:5])[0]
                        cmd_id = struct.unpack_from("<H", packet[5:7])[0]
                        data = packet[7:7+data_length]
                        frame_tail = struct.unpack_from("<H", packet[7+data_length:9+data_length])[0]
                        if self.print_info_RX:
                            info_to_print = "--------------------------------\n"
                            info_to_print += (f"RX data_length: {data_length}\n")
                            info_to_print += (f"RX seq: {seq}\n")
                            if CRC8 == Get_CRC8_Check_Sum(frame_header, 4):
                                info_to_print += ("RX CRC8 Pass\n")
                            else:
                                info_to_print += ("RX CRC8 Error\n")
                            info_to_print += (f"RX cmd_id: 0x{cmd_id:04X}\n")
                            info_to_print += (f"RX data: {data}\n")
                            if frame_tail == Get_CRC16_Check_Sum(packet, 7+data_length):
                                info_to_print += ("RX frame_tail Pass")
                            else:
                                info_to_print += ("RX frame_tail Error")
                            print(info_to_print)
                        self.real_serial_data_temp = self.real_serial_data_temp[9+data_length:]
                        has_SOF_flag = True
                        break
            
            return real_serial_data
        else:
            return b''