import threading
import time
from collections import deque
import serial

import sys, os
sys.path.append(os.path.dirname(__file__))

from RM_serial_py.ser_api import  build_send_packet, receive_packet, Radar_decision, \
    build_data_decision, build_data_radar_all

from FakeSerial import FakeSerial_Radar

class Communicator:
    def __init__(self, state = 'B', debug = False):
        self.state = state  # R:红方/B:蓝方
        # 初始化战场信息UI（标记进度、双倍易伤次数、双倍易伤触发状态）
        self.double_vulnerability_chance = -1  # 双倍易伤机会数
        self.opponent_double_vulnerability = -1  # 是否正在触发双倍易伤
        self.target = -1  # 飞镖当前瞄准目标（用于触发双倍易伤）
        self.chances_flag = 1  # 双倍易伤触发标志位，需要从1递增，每小局比赛会重置，所以每局比赛要重启程序
        self.progress_list = [-1, -1, -1, -1, -1, -1]  # 标记进度列表
        # 初始化盲区预测列表
        self.guess_list = {
            "B1": True,
            "B2": True,
            "B3": True,
            "B4": True,
            "B5": True,
            "B6": True,
            "B7": True,
            "R1": True,
            "R2": True,
            "R3": True,
            "R4": True,
            "R5": True,
            "R6": True,
            "R7": True
        }
        # 上次盲区预测时的标记进度
        self.guess_value = {
            "B1": 0,
            "B2": 0,
            "B7": 0,
            "R1": 0,
            "R2": 0,
            "R7": 0
        }
        # 当前标记进度（用于判断是否预测正确正确）
        self.guess_value_now = {
            "B1": 0,
            "B2": 0,
            "B7": 0,
            "R1": 0,
            "R2": 0,
            "R7": 0
        }
        # 机器人名字对应ID
        self.mapping_table = {
            "R1": 1,
            "R2": 2,
            "R3": 3,
            "R4": 4,
            "R5": 5,
            "R6": 6,
            "R7": 7,
            "B1": 101,
            "B2": 102,
            "B3": 103,
            "B4": 104,
            "B5": 105,
            "B6": 106,
            "B7": 107
        }
        # 盲区预测点位，如果没有定位模块，连接数服务器的非哨兵机器人坐标为（0,0）
        self.guess_table = {
            "R1": [(1100, 1400), (900, 1400)],
            "R2": [(870, 1100), (1340, 680)],
            "R7": [(560, 630), (560, 870)],
            "B1": [(1700, 100), (1900, 100)],
            # "B1": [(0, 0), (19.0, 1.0)],
            "B2": [(1930, 400), (1460, 820)],
            "B7": [(2240, 870), (2240, 603)],
            # "B7": [(0, 0), (22.4, 6.3)]
        }
        if not debug:
            self.ser1 = serial.Serial('COM1', 115200, timeout=1)  # 串口，替换 'COM1' 为你的串口号
        else:
            self.ser1 = FakeSerial_Radar()
        self.filter = Filter(window_size=3, communicator=self, max_inactive_time=2)
        self.start_serial()

    def start_serial(self):
        # 串口接收线程
        thread_receive = threading.Thread(target=self.ser_receive, daemon=True)
        thread_receive.start()
        # 串口发送线程
        thread_list = threading.Thread(target=self.ser_send, daemon=True)
        thread_list.start()

    # 串口发送线程
    def ser_send(self):
        seq = 0
        # 单点预测时间
        guess_time = {
            'B1': 0,
            'B2': 0,
            'B7': 0,
            'R1': 0,
            'R2': 0,
            'R7': 0,
        }
        # 预测点索引
        guess_index = {
            'B1': 0,
            'B2': 0,
            'B7': 0,
            'R1': 0,
            'R2': 0,
            'R7': 0,
        }
        # 发送机器人坐标
        def send_point(send_name, all_filter_data):
            # front_time = time.time()
            # 转换为地图坐标系
            filtered_xyz = (all_filter_data[send_name][0], all_filter_data[send_name][1])
            # 转换为裁判系统单位cm （mm to cm）
            ser_x = int(filtered_xyz[0]) / 10
            ser_y = int(filtered_xyz[1]) / 10
            # 打包坐标数据包
            # data = build_data_radar(self.mapping_table.get(send_name), ser_x, ser_y)
            # packet, seq_s = build_send_packet(data, seq_s, [0x03, 0x05])
            # self.ser1.write(packet)
            # back_time = time.time()
            # # 计算发送延时，动态调整
            # waste_time = back_time - front_time
            # # print("发送：",send_name, seq_s)
            # time.sleep(0.1 - waste_time)
            return ser_x,ser_y
        # 发送盲区预测点坐标
        def send_point_guess(send_name, guess_time_limit):
            # front_time = time.time()
            # print(self.guess_value_now.get(send_name),self.guess_value.get(send_name) ,guess_index[send_name])
            # 进度未满 and 预测进度没有涨 and 超过单点预测时间上限，同时满足则切换另一个点预测
            if self.guess_value_now.get(send_name) < 120 and self.guess_value_now.get(send_name) - self.guess_value.get(
                    send_name) <= 0 and time.time() - guess_time.get(send_name) >= guess_time_limit:
                guess_index[send_name] = 1 - guess_index[send_name]  # 每个ID不一样
                guess_time[send_name] = time.time()
            if self.guess_value_now.get(send_name) - self.guess_value.get(send_name) > 0:
                guess_time[send_name] = time.time()
            # 打包坐标数据包
            # data = build_data_radar(self.mapping_table.get(send_name), self.guess_table.get(send_name)[guess_index.get(send_name)][0],
            #                         self.guess_table.get(send_name)[guess_index.get(send_name)][1])
            # packet, seq_s = build_send_packet(data, seq_s, [0x03, 0x05])
            # self.ser1.write(packet)
            # back_time = time.time()
            # 计算发送延时，动态调整
            # waste_time = back_time - front_time
            # print('发送：',send_name, seq_s)
            # time.sleep(0.1 - waste_time)
            return self.guess_table.get(send_name)[guess_index.get(send_name)][0],self.guess_table.get(send_name)[guess_index.get(send_name)][1]
        time_s = time.time()
        target_last = 0  # 上一帧的飞镖目标
        update_time = 0  # 上次预测点更新时间
        send_count = 0  # 信道占用数，上限为4
        send_map = {
            "R1": (0, 0),
            "R2": (0, 0),
            "R3": (0, 0),
            "R4": (0, 0),
            "R5": (0, 0),
            "R6": (0, 0),
            "R7": (0, 0),
            "B1": (0, 0),
            "B2": (0, 0),
            "B3": (0, 0),
            "B4": (0, 0),
            "B5": (0, 0),
            "B6": (0, 0),
            "B7": (0, 0)
        }
        while True:
            guess_time_limit = 3 + 1.7  # 单位：秒，根据上一帧的信道占用数动态调整单点预测时间
            # print(guess_time_limit)
            send_count = 0  # 重置信道占用数
            try:
                all_filter_data = self.filter.get_all_data()
                if self.state == 'R':
                    if not self.guess_list.get('B1'):
                        if all_filter_data.get('B1', False):
                            send_map['B1'] = send_point('B1', all_filter_data)
                    else:
                        send_map['B1'] = (0, 0)

                    if not self.guess_list.get('B2'):
                        if all_filter_data.get('B2', False):
                            send_map['B2'] = send_point('B2',  all_filter_data)
                    else:
                        send_map['B2'] = (0, 0)

                    # 步兵3号
                    if not self.guess_list.get('B3'):
                        if all_filter_data.get('B3', False):
                            send_map['B3'] = send_point('B3',  all_filter_data)
                    else:
                        send_map['B3'] = (0, 0)

                    # 步兵4号
                    if not self.guess_list.get('B4'):
                        if all_filter_data.get('B4', False):
                            send_map['B4'] = send_point('B4', all_filter_data)
                    else:
                        send_map['B4'] = (0, 0)

                    if not self.guess_list.get('B5'):
                        if all_filter_data.get('B5', False):
                            send_map['B5'] = send_point('B5',  all_filter_data)
                    else:
                        send_map['B5'] = (0, 0)

                    # 哨兵
                    if self.guess_list.get('B7'):
                        send_map['B7'] = send_point_guess('B7', guess_time_limit)
                    # 未识别到哨兵，进行盲区预测
                    else:
                        if all_filter_data.get('B7', False):
                            send_map['B7'] = send_point('B7', all_filter_data)
                if self.state == 'B':
                    if not self.guess_list.get('R1'):
                        if all_filter_data.get('R1', False):
                            send_map['R1'] = send_point('R1', all_filter_data)
                    else:
                        send_map['R1'] = (0, 0)

                    if not self.guess_list.get('R2'):
                        if all_filter_data.get('R2', False):
                            send_map['R2'] = send_point('R2', all_filter_data)
                    else:
                        send_map['R2'] = (0, 0)

                    # 步兵3号
                    if not self.guess_list.get('R3'):
                        if all_filter_data.get('R3', False):
                            send_map['R3'] = send_point('R3', all_filter_data)
                    else:
                        send_map['R3'] = (0, 0)

                    # 步兵4号
                    if not self.guess_list.get('R4'):
                        if all_filter_data.get('R4', False):
                            send_map['R4'] = send_point('R4', all_filter_data)
                    else:
                        send_map['R4'] = (0, 0)

                    if not self.guess_list.get('R5'):
                        if all_filter_data.get('R5', False):
                            send_map['R5'] = send_point('R5', all_filter_data)
                    else:
                        send_map['R5'] = (0, 0)

                    # 哨兵
                    if self.guess_list.get('R7'):
                        send_map['R7'] = send_point_guess('R7', guess_time_limit)
                    # 未识别到哨兵，进行盲区预测
                    else:
                        if all_filter_data.get('R7', False):
                            send_map['R7'] = send_point('R7', all_filter_data)
                ser_data = build_data_radar_all(send_map,self.state)
                packet, seq = build_send_packet(ser_data, seq, [0x03, 0x05])
                self.ser1.write(packet)
                time.sleep(0.2)
                # print(send_map,seq)
                # 超过单点预测时间上限，更新上次预测的进度
                if time.time() - update_time > guess_time_limit:
                    update_time = time.time()
                    if self.state == 'R':
                        self.guess_value['B1'] = self.guess_value_now.get('B1')
                        self.guess_value['B2'] = self.guess_value_now.get('B2')
                        self.guess_value['B7'] = self.guess_value_now.get('B7')
                    else:
                        self.guess_value['R1'] = self.guess_value_now.get('R1')
                        self.guess_value['R2'] = self.guess_value_now.get('R2')
                        self.guess_value['R7'] = self.guess_value_now.get('R7')
                # 判断飞镖的目标是否切换，切换则尝试发动双倍易伤
                if self.target != target_last and self.target != 0:
                    target_last = self.target
                    # 有双倍易伤机会，并且当前没有在双倍易伤
                    if self.double_vulnerability_chance > 0 and self.opponent_double_vulnerability == 0:
                        time_e = time.time()
                        # 发送时间间隔为10秒
                        if time_e - time_s > 10:
                            print("请求双倍触发")
                            data = build_data_decision(self.chances_flag, self.state)
                            packet, seq = build_send_packet(data, seq, [0x03, 0x01])
                            # print(packet.hex(),self.chances_flag,self.state)
                            self.ser1.write(packet)
                            print("请求成功", self.chances_flag)
                            # 更新标志位
                            self.chances_flag += 1
                            if self.chances_flag >= 3:
                                self.chances_flag = 1
                            time_s = time.time()
            except Exception as r:
                print('未知错误 %s' % (r))

    # 裁判系统串口接收线程
    def ser_receive(self):
        progress_cmd_id = [0x02, 0x0C]  # 任意想要接收数据的命令码，这里是雷达标记进度的命令码0x020E
        vulnerability_cmd_id = [0x02, 0x0E]  # 双倍易伤次数和触发状态
        target_cmd_id = [0x01, 0x05]  # 飞镖目标
        buffer = b''  # 初始化缓冲区
        while True:
            # 从串口读取数据
            received_data = self.ser1.read_all()  # 读取一秒内收到的所有串口数据
            # 将读取到的数据添加到缓冲区中
            buffer += received_data

            # 查找帧头（SOF）的位置
            sof_index = buffer.find(b'\xA5')

            while sof_index != -1:
                # 如果找到帧头，尝试解析数据包
                if len(buffer) >= sof_index + 5:  # 至少需要5字节才能解析帧头
                    # 从帧头开始解析数据包
                    packet_data = buffer[sof_index:]

                    # 查找下一个帧头的位置
                    next_sof_index = packet_data.find(b'\xA5', 1)

                    if next_sof_index != -1:
                        # 如果找到下一个帧头，说明当前帧头到下一个帧头之间是一个完整的数据包
                        packet_data = packet_data[:next_sof_index]
                        # print(packet_data)
                    else:
                        # 如果没找到下一个帧头，说明当前帧头到末尾不是一个完整的数据包
                        break

                    # 解析数据包
                    progress_result = receive_packet(packet_data, progress_cmd_id,
                                                    info=False)  # 解析单个数据包，cmd_id为0x020E,不输出日志
                    vulnerability_result = receive_packet(packet_data, vulnerability_cmd_id, info=False)
                    target_result = receive_packet(packet_data, target_cmd_id, info=False)
                    # 更新裁判系统数据，标记进度、易伤、飞镖目标
                    if progress_result is not None:
                        received_cmd_id1, received_data1, received_seq1 = progress_result
                        self.progress_list = list(received_data1)
                        if self.state == 'R':
                            self.guess_value_now['B1'] = self.progress_list[0]
                            self.guess_value_now['B2'] = self.progress_list[1]
                            self.guess_value_now['B7'] = self.progress_list[5]
                        else:
                            self.guess_value_now['R1'] = self.progress_list[0]
                            self.guess_value_now['R2'] = self.progress_list[1]
                            self.guess_value_now['R7'] = self.progress_list[5]
                    if vulnerability_result is not None:
                        received_cmd_id2, received_data2, received_seq2 = vulnerability_result
                        received_data2 = list(received_data2)[0]
                        self.double_vulnerability_chance, self.opponent_double_vulnerability = Radar_decision(received_data2)
                    if target_result is not None:
                        received_cmd_id3, received_data3, received_seq3 = target_result
                        self.target = (list(received_data3)[1] & 0b1100000) >> 5

                    # 从缓冲区中移除已解析的数据包
                    buffer = buffer[sof_index + len(packet_data):]

                    # 继续寻找下一个帧头的位置
                    sof_index = buffer.find(b'\xA5')

                else:
                    # 缓冲区中的数据不足以解析帧头，继续读取串口数据
                    break
            time.sleep(0.5)
    
    def add_data(self, name, x, y): # 单位 mm
        self.filter.add_data(name, x, y)

# 机器人坐标滤波器（滑动窗口均值滤波）
class Filter:
    def __init__(self, window_size, communicator, max_inactive_time=2.0):
        self.window_size = window_size
        self.max_inactive_time = max_inactive_time
        self.data = {}  # 存储不同机器人的数据
        self.window = {}  # 存储滑动窗口内的数据
        self.last_update = {}  # 存储每个机器人的最后更新时间
        self.communicator = communicator

    # 添加机器人坐标数据
    def add_data(self, name, x, y):  # 删除阈值过滤机制
        if name not in self.data:
            # 如果实体名称不在数据字典中，初始化相应的deque。
            self.data[name] = deque(maxlen=self.window_size)
            self.window[name] = deque(maxlen=self.window_size)

        # 将坐标数据添加到数据字典和滑动窗口中。
        self.data[name].append((x, y))
        self.communicator.guess_list[name] = False

        self.window[name].append((x, y))
        self.last_update[name] = time.time()  # 更新最后更新时间

    # 过滤计算滑动窗口平均值
    def filter_data(self, name):
        if name not in self.data:
            return None

        if len(self.window[name]) < self.window_size:
            return None  # 不足以进行滤波

        # 计算滑动窗口内的坐标平均值
        x_avg = sum(coord[0] for coord in self.window[name]) / self.window_size
        y_avg = sum(coord[1] for coord in self.window[name]) / self.window_size

        return x_avg, y_avg

    # 获取所有机器人坐标
    def get_all_data(self):
        filtered_d = {}
        for name in self.data:
            # 超过max_inactive_time没识别到机器人将会清空缓冲区，并进行盲区预测
            if time.time() - self.last_update[name] > self.max_inactive_time:
                self.data[name].clear()
                self.window[name].clear()
                self.communicator.guess_list[name] = True
            # 识别到机器人，不进行盲区预测
            else:
                self.communicator.guess_list[name] = False
                filtered_d[name] = self.filter_data(name)
        # 返回所有当前识别到的机器人及其坐标的均值
        return filtered_d



if __name__ == "__main__":

    communicator = Communicator(True)
    communicator.start_serial()
    cls, X_M, Y_M = "R7",500,1000

    while True:
        communicator.add_data(cls, X_M, Y_M)
