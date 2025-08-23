import ctypes
import numpy as np
import cv2

import os, sys
sys.path.append(os.path.dirname(__file__))

from MvImport.CameraParams_header import MV_CC_DEVICE_INFO_LIST, MV_CC_DEVICE_INFO, MV_FRAME_OUT_INFO_EX, MV_CC_DEVICE_INFO_LIST
from MvImport.CameraParams_const import MV_USB_DEVICE, MV_ACCESS_Exclusive
from MvImport.MvCameraControl_class import MvCamera



class Camera:
    def __init__(self, camera_index=0):
        self._deviceList = MV_CC_DEVICE_INFO_LIST()
        self._tlayerType = MV_USB_DEVICE
        self._cam = MvCamera()
        self._camera_index = camera_index
        self._width = 4024
        self._height = 3036

        # 枚举设备
        ret = MvCamera.MV_CC_EnumDevices(self._tlayerType, self._deviceList)
        if ret != 0 or self._deviceList.nDeviceNum == 0:
            print("未找到设备！")
            sys.exit()

        # 创建句柄并打开设备
        stDeviceList = ctypes.cast(self._deviceList.pDeviceInfo[camera_index], ctypes.POINTER(MV_CC_DEVICE_INFO)).contents
        ret = self._cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            print(f"创建句柄失败，错误码: {ret}")
            sys.exit()

        ret = self._cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print(f"打开设备失败，错误码: {ret}")
            sys.exit()
            
    def set_exposure_time(self, exposure_time):
        """设置曝光时间（微秒）"""
        ret = self._cam.MV_CC_SetFloatValue("ExposureTime", exposure_time)
        if ret != 0:
            print(f"设置曝光时间失败，错误码: {ret}")
        else:
            print(f"曝光时间已设置为: {exposure_time} 微秒")

    def set_gain(self, gain):
        """设置增益（dB）"""
        ret = self._cam.MV_CC_SetFloatValue("Gain", gain)
        if ret != 0:
            print(f"设置增益失败，错误码: {ret}")
        else:
            print(f"增益已设置为: {gain} dB")

    def start_grabbing(self):
        # 开始取流
        ret = self._cam.MV_CC_StartGrabbing()
        if ret != 0:
            print(f"开始取流失败，错误码: {ret}")
            sys.exit()

    def get_image(self):
        # 获取一帧图像数据
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        
        # Bayer格式每个像素占1字节
        buffer_size = self._width * self._height
        data_buf = (ctypes.c_ubyte * buffer_size)()

        ret = self._cam.MV_CC_GetOneFrameTimeout(ctypes.byref(data_buf), buffer_size, stFrameInfo, 1000)
        if ret == 0:
            # Bayer格式数据，需要转换为RGB
            bayer_data = np.frombuffer(data_buf, dtype=np.uint8).reshape((self._height, self._width))
            
            # 将BayerRG8转换为BGR（OpenCV默认格式）
            # 注意：这里使用的是RG Bayer模式，请根据实际相机传感器类型调整
            img_bgr = cv2.cvtColor(bayer_data, cv2.COLOR_BayerBG2BGR)
            
            return img_bgr
        else:
            print(f"获取图像失败，错误码: {ret}")
            return None

    def stop_grabbing(self):
        # 停止取流并关闭设备
        self._cam.MV_CC_StopGrabbing()
        self._cam.MV_CC_CloseDevice()
