import struct

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

    def print_infos(self):
        info_to_print = ""
        info_to_print += "发送数据解析:(单位cm)"
        info_to_print += f"|英雄:({self.hero_position_x}, {self.hero_position_y})"
        info_to_print += f"|工程:({self.engineer_position_x}, {self.engineer_position_y})"
        info_to_print += f"|3号步兵:({self.infantry_3_position_x}, {self.infantry_3_position_y})"
        info_to_print += f"|4号步兵:({self.infantry_4_position_x}, {self.infantry_4_position_y})"
        info_to_print += f"|5号步兵:({self.infantry_5_position_x}, {self.infantry_5_position_y})"
        info_to_print += f"|哨兵:({self.sentry_position_x}, {self.sentry_position_y})"
        print(info_to_print)


class FakeSerial_Radar:
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
        map_robot_data_py.print_infos()
    def read_all(self):
        return b''