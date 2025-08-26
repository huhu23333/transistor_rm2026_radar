import ctypes
import numpy as np
import os
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import threading
import time
import cv2



class PYIF_PonitData(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_int32),
        ("y", ctypes.c_int32),
        ("z", ctypes.c_int32),
        ("reflectivity", ctypes.c_uint8),
        ("tag", ctypes.c_uint8)
    ]


class LivoxInterface:
    def __init__(self):
        self.integrate_time = 300 # ms （桑佰毫秒，够吗？应该够吧。来吧，试下米↓↑）
        self.PYIF_PonitDataArrayLen = 4096
        self.image2dSize = 1024
        self.position_bias = (0, 0, 0)
        self.integrateTemp = []
        self.integrateTempTimes = []
        self.integrateResult = np.zeros((0, 5))
        self.integrateResult_rad = np.zeros((0, 5))
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
        self.image2dResult = [np.zeros((self.image2dSize, self.image2dSize, 3), dtype=np.uint8),
                              np.zeros((self.image2dSize, self.image2dSize, 1), dtype=np.uint8)]

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
    
    def xyzToYawPitchR(self, xs, ys, zs, others=[]):
        rs = np.sqrt(np.square(xs) + np.square(ys) + np.square(zs))
        yaws = np.arctan2(-ys, xs)
        pitchs = np.arcsin(zs / rs)
        results = np.column_stack([yaws, pitchs, rs, *others]).reshape(-1, 3+len(others))
        return results

    def updatingThreadFunc(self):
        while self.py_updatingResult:
            if self.py_readTo == 0:
                xyzrt = self.extractXYZRT(self.readArray0())
                xyzrt[:,0] += self.position_bias[0]
                xyzrt[:,1] += self.position_bias[1]
                xyzrt[:,2] += self.position_bias[2]
                self.integrateTemp.append(xyzrt)
                self.integrateTempTimes.append(time.time())
                self.py_readTo = 1
            elif self.py_readTo == 1:
                xyzrt = self.extractXYZRT(self.readArray1())
                xyzrt[:,0] += self.position_bias[0]
                xyzrt[:,1] += self.position_bias[1]
                xyzrt[:,2] += self.position_bias[2]
                self.integrateTemp.append(xyzrt)
                self.integrateTempTimes.append(time.time())
                self.py_readTo = 0
            while (len(self.integrateTempTimes) > 0) and ((time.time() - self.integrateTempTimes[0]) * 1000 > self.integrate_time):
                self.integrateTemp = self.integrateTemp[1:]
                self.integrateTempTimes = self.integrateTempTimes[1:]
            self.integrateResult = np.stack(self.integrateTemp).reshape(-1, 5)
            self.integrateResult_rad = self.xyzToYawPitchR(self.integrateResult[:,0], self.integrateResult[:,1], self.integrateResult[:,2], 
                                                           [self.integrateResult[:,3], self.integrateResult[:,4]])

    def startUpdatingThread(self):
        self.py_updatingResult = True
        self.updatingThread = threading.Thread(target=self.updatingThreadFunc, daemon=True)
        self.updatingThread.start()

    def endUpdatingThread(self):
        self.py_updatingResult = False

    def get_nearest_points(self, rad_center, number, return_valve = "ypr"):
        integrateResult_temp = self.integrateResult.copy()
        integrateResult_rad_temp = self.integrateResult_rad.copy()
        temp_size = min(len(integrateResult_temp), len(integrateResult_rad_temp))
        if temp_size == 0:
            return np.zeros((number,5))
        integrateResult_temp = integrateResult_temp[:temp_size]
        integrateResult_rad_temp = integrateResult_rad_temp[:temp_size]
        return_size = min(temp_size, number)
        min_distance_indexes = np.argsort(np.sqrt(np.square(integrateResult_rad_temp[:,0]-rad_center[0]) + np.square(integrateResult_rad_temp[:,1]-rad_center[1])))[:return_size]
        if return_valve == "ypr":
            return integrateResult_rad_temp[min_distance_indexes]
        else:
            return integrateResult_temp[min_distance_indexes]

    def get_points_in_range(self, rad_center, rad_half_w, rad_half_h, method = "L1", return_valve = "ypr"): # L1 L2 L-infinity
        integrateResult_temp = self.integrateResult.copy()
        integrateResult_rad_temp = self.integrateResult_rad.copy()
        temp_size = min(len(integrateResult_temp), len(integrateResult_rad_temp))
        if temp_size == 0:
            return self.get_nearest_points(rad_center, 100, return_valve)
        integrateResult_temp = integrateResult_temp[:temp_size]
        integrateResult_rad_temp = integrateResult_rad_temp[:temp_size]
        if method == "L1":
            mask = np.abs(integrateResult_rad_temp[:,0]-rad_center[0])/rad_half_w + np.abs(integrateResult_rad_temp[:,1]-rad_center[1])/rad_half_h <= 1
            if not np.any(mask):
                return self.get_nearest_points(rad_center, 100, return_valve)
            return integrateResult_rad_temp[mask] if return_valve == "ypr" else integrateResult_temp[mask]
        elif method == "L2":
            mask = np.sqrt(np.square(integrateResult_rad_temp[:,0]-rad_center[0])/(rad_half_w**2) + np.square(integrateResult_rad_temp[:,1]-rad_center[1])/(rad_half_h**2)) <= 1
            if not np.any(mask):
                return self.get_nearest_points(rad_center, 100, return_valve)
            return integrateResult_rad_temp[mask] if return_valve == "ypr" else integrateResult_temp[mask]
        elif method == "L-infinity":
            mask = np.logical_and(np.abs(integrateResult_rad_temp[:,0]-rad_center[0]) <= rad_half_w , np.abs(integrateResult_rad_temp[:,1]-rad_center[1]) <= rad_half_h)
            if not np.any(mask):
                return self.get_nearest_points(rad_center, 100, return_valve)
            return integrateResult_rad_temp[mask] if return_valve == "ypr" else integrateResult_temp[mask]
        else: # 默认使用L1距离
            mask = np.abs(integrateResult_rad_temp[:,0]-rad_center[0])/rad_half_w + np.abs(integrateResult_rad_temp[:,1]-rad_center[1])/rad_half_h <= 1
            if not np.any(mask):
                return self.get_nearest_points(rad_center, 100, return_valve)
            return integrateResult_rad_temp[mask] if return_valve == "ypr" else integrateResult_temp[mask]
    
    # -------------------------以下为测试代码---------------------------------

    def draw2dImage(self):
        image = np.zeros((self.image2dSize, self.image2dSize, 1), dtype=np.uint8)
        fimages, mask = self.pyif_draw2dImageF(self.integrateResult_rad[:,0], self.integrateResult_rad[:,1], 
                                                [self.integrateResult_rad[:,2], self.integrateResult_rad[:,3]])
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

        self.image2dResult = [image, mask]
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
            cv2.imshow("Livox test", self.draw2dImage())
            #print(f"{self.py_readCount} | {self.pyif_writeCount.value} | {len(self.integrateResult)}")
            if cv2.waitKey(1) & 0xFF == 27:  # 按下 ESC 键退出
                break
