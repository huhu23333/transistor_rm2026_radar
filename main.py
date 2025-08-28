import os, sys
sys.path.append(os.path.dirname(__file__))
from LivoxCtypesInterface import LivoxInterface
from Camera import Camera
from CoordinateConversions import pixToRad, radToPix, newPixToRad, radToNewPix
import threading
import numpy as np
import math
import cv2
import random
from ultralytics import YOLO
from PIL import Image
from YOLOResultsFilter import remove_overlapping_boxes

from CoordinateConversions import mapToRad, debug_rand_octagon, debug_yolo_to_distance



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
    car_conf_threshold = 0.25
    model = YOLO(model_path)
    model.export(format="openvino",int8=True) """
    car_model_path = "rmCar_yolov12n_int8_openvino_model"
    car_conf_threshold = 0.25
    car_model = YOLO(car_model_path)

    armor_model_path = "rmArmor_yolov12n_int8_openvino_model"
    armor_conf_threshold = 0.5
    armor_model = YOLO(armor_model_path)
    #armor_model.export(format="openvino",int8=True)

    while True:

        img = camera.get_image()
        if img is not None:
            car_model_results = car_model(img, conf=car_conf_threshold, verbose=False)

            # 去除重叠检测框，保留较大的
            keep_indices = remove_overlapping_boxes(car_model_results[0].boxes)
            remove_count = len(car_model_results[0].boxes) - len(keep_indices)
            car_model_results[0].boxes = car_model_results[0].boxes[keep_indices]
            """ if remove_count:
                print(f"removed {remove_count} boxes") """

            # 处理每个检测框
            custom_car_boxes = []
            for carBox in car_model_results[0].boxes:
                custom_car_boxes.append({"box": carBox})
                x1, y1, x2, y2 = map(int, carBox.xyxy[0])
                # 提取边界框区域
                car_box_region = img[y1:y2, x1:x2]
                # 在这里进行图像处理
                armor_model_results = armor_model(car_box_region, conf=armor_conf_threshold, verbose=False)
                annotated_img_armor = armor_model_results[0].plot()
                clsCount = [0]*12
                for armorBox in armor_model_results[0].boxes:
                    clsCount[int(armorBox.cls)] += 1
                if sum(clsCount) == 0:
                    custom_car_boxes[-1]["cls"] = 12
                else:
                    custom_car_boxes[-1]["cls"] = clsCount.index(max(clsCount))
                # 将处理后的区域放回原图
                img[y1:y2, x1:x2] = annotated_img_armor

            annotated_img = car_model_results[0].plot()

            """ for box in car_model_results[0].boxes:
                print(box) """
            
            img_radar = livoxInterface.image2dResult[0]
            img_radar_mask = livoxInterface.image2dResult[1]
            img_mapToRad = cv2.resize(mapToRad(annotated_img), (1024, 1024))
            cv2.copyTo(img_radar, img_radar_mask, img_mapToRad)

            #debug_rand_octagon(annotated_img, img_mapToRad, livoxInterface)
            debug_yolo_to_distance(annotated_img, img_mapToRad, livoxInterface, custom_car_boxes)

            #annotated_img = cv2.resize(annotated_img, (1006, 759))
            cv2.imshow("Camera", annotated_img)
            cv2.imshow("Camera mapToRad", img_mapToRad)

        if cv2.waitKey(1) & 0xFF == 27:  # 按下 ESC 键退出
            break
    camera.stop_grabbing()
    #cv2.destroyAllWindows()



if __name__ == "__main__":
    main()
