import os, sys
sys.path.append(os.path.dirname(__file__))
from LivoxCtypesInterface import LivoxInterface
from Camera import Camera
import threading
import numpy as np
import math
import cv2
import time
import random

""" 内参矩阵 K:
 [[3.24060091e+03 0.00000000e+00 2.03544211e+03]
 [0.00000000e+00 3.24406649e+03 1.55545699e+03]
 [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
畸变系数 dist_coef:
 [[-0.21991307  0.48251945 -0.00198696  0.00373789 -0.42319698]] """
# 相机内参矩阵和畸变系数
K = np.array([[3.24060091e+03, 0,  2.03544211e+03],
              [0,  3.24406649e+03, 1.55545699e+03],
              [0,  0,  1.00000000e+00]], dtype=np.float32)
D = np.array([-0.21991307, 0.48251945, -0.00198696, 0.00373789, -0.42319698], dtype=np.float32)

def pixToRad(u, v): # 原像素点坐标
    # 去畸变后的像素点坐标
    undistorted_points = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float32), K, D)
    # 获取去畸变后的规范化坐标
    x_prime, y_prime = undistorted_points[0][0]
    # 计算Yaw和Pitch
    yaw_rad = np.arctan2(x_prime, 1)
    pitch_rad = np.arctan2(-y_prime, np.sqrt(x_prime**2 + 1))
    #print(np.array([yaw_rad, pitch_rad]))
    return np.array([yaw_rad, pitch_rad])

def radToPix(yaw_rad, pitch_rad):
    # 计算规范化坐标
    x_prime = np.tan(yaw_rad)
    y_prime = -np.tan(pitch_rad)*np.sqrt(x_prime**2 + 1)
    # 转换为齐次坐标
    norm_coords = np.array([[x_prime, y_prime, 1]], dtype=np.float32)
    # 通过内参矩阵将规范化坐标转化为像素坐标
    pixel_coords = cv2.projectPoints(norm_coords, np.zeros(3), np.zeros(3), K, D)[0]
    # 获取像素坐标 (u, v)
    u, v = pixel_coords[0][0]
    return u, v

rad_bais = [0.010, 0.026] # 增加值将摄像机图像左移、下移
def newPixToRad(u, v, image_size = (1024, 1024)):
    return (u/image_size[0]-0.5)*1.2287117934040082+rad_bais[0], -(v/image_size[0]-0.5)*1.2287117934040082+rad_bais[1]

def radToNewPix(yaw_rad, pitch_rad, image_size = (1024, 1024), toInt = False):
    result = ((yaw_rad-rad_bais[0])/1.2287117934040082+0.5)*image_size[0], (-(pitch_rad-rad_bais[1])/1.2287117934040082+0.5)*image_size[1]
    return (int(result[0]), int(result[1])) if toInt else result

map_x = np.zeros((1024, 1024), dtype=np.float32)
map_y = np.zeros((1024, 1024), dtype=np.float32)
print("!!!")
for y in range(1024):
    for x in range(1024):
        x_origin, y_origin = radToPix(*newPixToRad(x, y))
        map_x[y, x] = x_origin
        map_y[y, x] = y_origin
    print(x_origin, y_origin)

def mapToRad(image):
    mapped_img = cv2.remap(image, map_x, map_y, interpolation=cv2.INTER_LINEAR)
    return mapped_img



def debug_util_center_to_vertexes(center):
    half_side_len = 100
    pts = np.array([
        [center[0]-half_side_len, center[1]-half_side_len],
        [center[0]-half_side_len, center[1]],
        [center[0]-half_side_len, center[1]+half_side_len],
        [center[0], center[1]+half_side_len],
        [center[0]+half_side_len, center[1]+half_side_len],
        [center[0]+half_side_len, center[1]],
        [center[0]+half_side_len, center[1]-half_side_len],
        [center[0], center[1]-half_side_len]
    ], dtype=np.int32)
    return pts


t_last_rand = time.time()
rand_posi = [random.randint(50,3974), random.randint(50,2986)]
def debug_rand_octagon(img1, img2, livoxInterface):
    global t_last_rand, rand_posi
    if time.time() - t_last_rand > 10:
        t_last_rand = time.time()
        rand_posi = [random.randint(50,3974), random.randint(50,2986)]
    pts = debug_util_center_to_vertexes(rand_posi)
    cv2.polylines(img1, [pts.reshape((-1, 1, 2))], isClosed=True, color=(0, 255, 0), thickness=5)
    new_pts = np.array([radToNewPix(*pixToRad(pt[0], pt[1])) for pt in pts], dtype=np.int32)
    cv2.polylines(img2, [new_pts.reshape((-1, 1, 2))], isClosed=True, color=(0, 255, 0), thickness=3)

    points_in_range = livoxInterface.get_points_in_range(pixToRad(*rand_posi), 0.03)
    print(np.median(points_in_range[:,2]))

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
    camera.set_gain(10.0)  # 设置增益为10dB
    
    camera.start_grabbing()

    while True:

        img = camera.get_image()
        if img is not None:
            img_radar = livoxInterface.image2dResult[0]
            img_radar_mask = livoxInterface.image2dResult[1]
            img_mapToRad = cv2.resize(mapToRad(img), (1024, 1024))
            cv2.copyTo(img_radar, img_radar_mask, img_mapToRad)

            debug_rand_octagon(img, img_mapToRad, livoxInterface)

            img = cv2.resize(img, (1006, 759))
            cv2.imshow("Camera", img)
            cv2.imshow("Camera mapToRad", img_mapToRad)

        if cv2.waitKey(1) & 0xFF == 27:  # 按下 ESC 键退出
            break
    camera.stop_grabbing()
    #cv2.destroyAllWindows()

    

if __name__ == "__main__":
    main()
