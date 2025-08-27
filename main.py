import os, sys
sys.path.append(os.path.dirname(__file__))
from LivoxCtypesInterface import LivoxInterface
from Camera import Camera
from utils import pixToRad, radToPix, newPixToRad, radToNewPix
from ArmorDetector import ArmorDetector
import threading
import numpy as np
import math
import cv2
import random
from ultralytics import YOLO
from PIL import Image

from utils import mapToRad, debug_rand_octagon, debug_yolo_to_distance





def main():
    livoxInterface = LivoxInterface()
    livoxInterface.pyif_Init()
    #livoxInterface.test()
    #livoxInterface.pyif_Uninit()
    livoxThread = threading.Thread(target=livoxInterface.test, daemon=True)
    livoxThread.start()

    camera = Camera(camera_index=0)
    # 设置相机参数
    camera.set_exposure_time(10000)  # 设置曝光时间为10000微秒
    camera.set_gain(16.0)  # 设置增益为10dB
    camera.start_grabbing()

    """ model_path = "rmCar_yolov12n.pt"
    conf_threshold = 0.25
    model = YOLO(model_path)
    model.export(format="openvino",int8=True) """
    car_model_path = "rmCar_yolov12n_int8_openvino_model"
    conf_threshold = 0.25
    car_model = YOLO(car_model_path)

    armorDetector = ArmorDetector()

    while True:

        img = camera.get_image()
        if img is not None:
            car_model_results = car_model(img, conf=conf_threshold, verbose=False)


            # 处理每个检测框
            for box in car_model_results[0].boxes.xyxy:
                x1, y1, x2, y2 = map(int, box)
                # 提取边界框区域
                box_region = img[y1:y2, x1:x2]
                # 在这里进行图像处理
                armorDetector.detectArmors(box_region)
                # 将处理后的区域放回原图
                # img[y1:y2, x1:x2] = box_region

            #armorDetector.detectArmors(img)
            annotated_img = car_model_results[0].plot()

            """ for box in car_model_results[0].boxes:
                print(box) """
            
            img_radar = livoxInterface.image2dResult[0]
            img_radar_mask = livoxInterface.image2dResult[1]
            img_mapToRad = cv2.resize(mapToRad(annotated_img), (1024, 1024))
            cv2.copyTo(img_radar, img_radar_mask, img_mapToRad)

            #debug_rand_octagon(annotated_img, img_mapToRad, livoxInterface)
            debug_yolo_to_distance(annotated_img, img_mapToRad, livoxInterface, car_model_results[0].boxes)

            annotated_img = cv2.resize(annotated_img, (1006, 759))
            cv2.imshow("Camera", annotated_img)
            cv2.imshow("Camera mapToRad", img_mapToRad)

        if cv2.waitKey(1) & 0xFF == 27:  # 按下 ESC 键退出
            break
    camera.stop_grabbing()
    #cv2.destroyAllWindows()



if __name__ == "__main__":
    main()
