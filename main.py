import ctypes
import numpy as np
import os, sys
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import threading
import time
import cv2

sys.path.append(os.path.dirname(__file__))
from MvImport.CameraParams_header import MV_CC_DEVICE_INFO_LIST, MV_CC_DEVICE_INFO, MV_FRAME_OUT_INFO_EX, MV_CC_DEVICE_INFO_LIST
from MvImport.CameraParams_const import MV_USB_DEVICE, MV_ACCESS_Exclusive
from MvImport.MvCameraControl_class import MvCamera
from MvImport.PixelType_header import PixelType_Gvsp_Mono8, PixelType_Gvsp_BGR8_Packed, PixelType_Gvsp_YUV422_Packed, PixelType_Gvsp_RGB8_Packed, PixelType_Gvsp_BayerRG8


class PYIF_PonitData(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_int32),
        ("y", ctypes.c_int32),
        ("z", ctypes.c_int32),
        ("reflectivity", ctypes.c_uint8),
        ("tag", ctypes.c_uint8)
    ]

class LivoxCtypesReader:
    def __init__(self):
        self.integrate_time = 300 # ms
        self.PYIF_PonitDataArrayLen = 4096
        self.image2dSize = 640
        self.integrateTemp = []
        self.integrateTempTimes = []
        self.integrateResult = np.zeros((0, 5))
        current_path = os.path.dirname(__file__)
        self.lib = ctypes.CDLL(os.path.join(current_path, "livox_python_interface.dll"))

        self.lib.pyif_Init.argtypes = []  # 设置参数类型
        self.lib.pyif_Init.restype = ctypes.c_int  # 设置返回类型

        self.lib.pyif_Uninit.argtypes = []  # 设置参数类型
        self.lib.pyif_Uninit.restype = None  # 设置返回类型

        self.lib.pyif_draw2dImageF.argtypes = [
            np.ctypeslib.ndpointer(ctypes.c_double, flags="C_CONTIGUOUS"),
            np.ctypeslib.ndpointer(ctypes.c_double, flags="C_CONTIGUOUS"),
            np.ctypeslib.ndpointer(ctypes.c_double, flags="C_CONTIGUOUS"),
            ctypes.c_uint64,
            ctypes.c_uint16,
            ctypes.c_uint16,
            np.ctypeslib.ndpointer(ctypes.c_double, flags="C_CONTIGUOUS"),
            np.ctypeslib.ndpointer(ctypes.c_uint8, flags="C_CONTIGUOUS")]
        self.lib.pyif_draw2dImageF.restype = None

        # 定义二维数组类型
        ArrayType = PYIF_PonitData * self.PYIF_PonitDataArrayLen * 2  # 2行，每行1000个元素
        # 获取全局变量并设置类型
        pyif_ponitDataArray = getattr(self.lib, "pyif_ponitDataArray")
        self.pyif_ponitDataArray_ptr = ctypes.cast(pyif_ponitDataArray, ctypes.POINTER(ArrayType))

        self.pyif_usedFlag_0 = ctypes.c_bool.in_dll(self.lib, "pyif_usedFlag_0")
        self.pyif_usedFlag_1 = ctypes.c_bool.in_dll(self.lib, "pyif_usedFlag_1")
        self.pyif_writeCount = ctypes.c_uint64.in_dll(self.lib, "pyif_writeCount")

        self.py_readTo = 0
        self.py_readCount = 0
        self.py_updatingResult = False

    def pyif_Init(self):
        result = self.lib.pyif_Init()
        return result

    def pyif_Uninit(self):
        self.lib.pyif_Uninit()
        
    def pyif_draw2dImageF(self, yaws, pitchs, fvalues, imageSize = None):
        if not imageSize:
            imageSize = self.image2dSize
        values_number = len(fvalues)
        results = np.zeros((values_number, imageSize, imageSize, 1), dtype=np.double)
        mask = np.zeros((imageSize, imageSize, 1), dtype=np.uint8)
        yaws = yaws.astype(np.double)
        pitchs = pitchs.astype(np.double)
        fvalues = np.stack(fvalues).astype(np.double)
        point_number = len(yaws)
        if any([point_number != target for target in [len(pitchs), *[len(fvalue) for fvalue in fvalues]]]):
            raise Exception
        self.lib.pyif_draw2dImageF(yaws, pitchs, fvalues, point_number, imageSize, values_number, results, mask)
        results = results.reshape(values_number, imageSize, imageSize, 1)
        return results, mask

    def readArray0(self):
        while self.pyif_usedFlag_0.value:
            time.sleep(0.001)
        np_array = np.ctypeslib.as_array(self.pyif_ponitDataArray_ptr.contents[0], shape=(self.PYIF_PonitDataArrayLen,)).copy()
        self.pyif_usedFlag_0.value = True
        self.py_readCount += self.PYIF_PonitDataArrayLen
        return np_array
    
    def readArray1(self):
        while self.pyif_usedFlag_1.value:
            time.sleep(0.001)
        np_array = np.ctypeslib.as_array(self.pyif_ponitDataArray_ptr.contents[1], shape=(self.PYIF_PonitDataArrayLen,)).copy()
        self.pyif_usedFlag_1.value = True
        self.py_readCount += self.PYIF_PonitDataArrayLen
        return np_array
    
    def extractXYZRT(self, np_array):
        xyzrt_array = np.column_stack((np_array['x'], np_array['y'], np_array['z'], np_array["reflectivity"], np_array["tag"])).astype(np.int32)
        return xyzrt_array

    def updatingThreadFunc(self):
        while self.py_updatingResult:
            if self.py_readTo == 0:
                self.integrateTemp.append(self.extractXYZRT(self.readArray0()))
                self.integrateTempTimes.append(time.time())
                self.py_readTo = 1
            elif self.py_readTo == 1:
                self.integrateTemp.append(self.extractXYZRT(self.readArray1()))
                self.integrateTempTimes.append(time.time())
                self.py_readTo = 0
            while (len(self.integrateTempTimes) > 0) and ((time.time() - self.integrateTempTimes[0]) * 1000 > self.integrate_time):
                self.integrateTemp = self.integrateTemp[1:]
                self.integrateTempTimes = self.integrateTempTimes[1:]
            self.integrateResult = np.stack(self.integrateTemp).reshape(-1, 5)

    def startUpdatingThread(self):
        self.py_updatingResult = True
        self.updatingThread = threading.Thread(target=self.updatingThreadFunc, daemon=True)
        self.updatingThread.start()

    def endUpdatingThread(self):
        self.py_updatingResult = False

    def xyzToYawPitchR(self, xs, ys, zs, others=[]):
        rs = np.sqrt(np.square(xs) + np.square(ys) + np.square(zs))
        yaws = np.atan2(-ys, xs)
        pitchs = np.asin(zs / rs)
        results = np.column_stack([yaws, pitchs, rs, *others]).reshape(-1, 3+len(others))
        return results
    
    def draw2dImage(self):
        yawPitchRs = self.xyzToYawPitchR(self.integrateResult[:,0], self.integrateResult[:,1], self.integrateResult[:,2], 
                                         [self.integrateResult[:,3]])
        image = np.zeros((self.image2dSize, self.image2dSize, 1), dtype=np.uint8)
        fimages, mask = self.pyif_draw2dImageF(yawPitchRs[:,0], yawPitchRs[:,1], 
                                                [yawPitchRs[:,2], yawPitchRs[:,3]])
        #r = (255-np.log(yawPitchR[2]+1)*25)*4
        #r = 255-yawPitchR[2]/100
        #r = 1000000 / yawPitchR[2]
        #r = 15000 / np.sqrt(yawPitchR[2])
        fimage_depth = fimages[0]
        image_depth = np.zeros_like(image)
        image_depth[mask==255] = np.clip(1000000 / np.clip(fimage_depth, 1, np.inf), 0, 255).astype(np.uint8)[mask==255]

        fimage_reflectivity = fimages[1]
        image_reflectivity = np.zeros_like(image)
        image_reflectivity[mask==255] = np.clip(fimage_reflectivity * 5, 0, 255).astype(np.uint8)[mask==255]

        brightness = (np.sqrt(image_reflectivity)*16).astype(np.uint8)
        hue = image_depth
        saturation = np.full_like(brightness, 255, dtype=np.uint8)

        # 将色相值从0-255映射到0-179（OpenCV HSV色彩空间的色相范围是0-179）
        hue_normalized = (hue.astype(np.float32) / 255.0) * 179.0
        hue_normalized = hue_normalized.astype(np.uint8)

        # 创建饱和度数组（这里使用全饱和，值为255）

        # 组合HSV数组（注意OpenCV中通道顺序是H,S,V）
        hsv_image = np.concatenate([hue_normalized, saturation, brightness], axis=2)

        # 将HSV转换为BGR（OpenCV默认使用BGR格式而不是RGB）
        bgr_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
        
        image = bgr_image
        #image = cv2.inpaint(image, 255-mask, 0, cv2.INPAINT_TELEA) # cv2.INPAINT_NS
        return image

    def visualizeNowReslut3d(self):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(elev=5, azim=-180)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        scatter = ax.scatter(self.integrateResult[:,0], self.integrateResult[:,1], self.integrateResult[:,2], c = self.integrateResult[:,3], alpha = 0.5, s=0.05) # creating new scatter chart with updated data
        plt.show()

    def test(self):
        self.startUpdatingThread()
        #for i in range(int(1e5)):
        #    print(f"{self.py_readCount} | {self.pyif_writeCount.value} | {len(self.integrateResult)}")
        #self.endUpdatingThread()
        #self.visualizeNowReslut3d()
        while True:
            cv2.imshow("", self.draw2dImage())
            print(f"{self.py_readCount} | {self.pyif_writeCount.value} | {len(self.integrateResult)}")
            cv2.waitKey(1)


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



def main():
    """ livoxReader = LivoxCtypesReader()
    livoxReader.pyif_Init()
    livoxReader.test()
    livoxReader.pyif_Uninit() """

    camera = Camera(camera_index=0)
    
    # 设置相机参数
    camera.set_exposure_time(10000)  # 设置曝光时间为10000微秒
    camera.set_gain(10.0)  # 设置增益为10dB
    
    camera.start_grabbing()
    while True:
        img = camera.get_image()
        if img is not None:
            img = cv2.resize(img, (1006, 759))
            cv2.imshow("Camera", img)

        if cv2.waitKey(1) & 0xFF == 27:  # 按下 ESC 键退出
            break
    camera.stop_grabbing()
    cv2.destroyAllWindows()

    

if __name__ == "__main__":
    main()
