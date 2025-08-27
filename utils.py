import random
import numpy as np
import time
import cv2


debug = True
team_color = "RED"


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
    return np.array([u, v])

rad_bais = [0.006, 0.023] # 增加值将摄像机图像左移、下移
def newPixToRad(u, v, image_size = (1024, 1024)):
    return np.array([(u/image_size[0]-0.5)*1.2287117934040082+rad_bais[0], -(v/image_size[0]-0.5)*1.2287117934040082+rad_bais[1]])

def radToNewPix(yaw_rad, pitch_rad, image_size = (1024, 1024)):
    result = np.array([((yaw_rad-rad_bais[0])/1.2287117934040082+0.5)*image_size[0], (-(pitch_rad-rad_bais[1])/1.2287117934040082+0.5)*image_size[1]])
    return result

def radToCamXYZ(yaw, pitch, r):
    x = r * np.cos(yaw) * np.cos(pitch)     # 向前
    y = r * -np.sin(yaw) * np.cos(pitch)    # 向左
    z = r * np.sin(pitch)                   # 向上
    return np.array([x, y, z])

cam_position_red = np.array([-1400, 5400, 2500+1200])     # x,y,z 向前,向左,向上 mm     使用rm25场地数据，假设雷达在雷达基座中心1.2m高支架上
cam_orientation_red = np.array([-0.13552771398550073, -0.23370661419387276, 0])  # yaw,pitch,roll 上视、左视、后视为顺时针 弧度制   假设画面中心对准场地中心地面
cam_position_blue = np.array([28000-cam_position_red[0], 15000-cam_position_red[1], cam_position_red[2]])
cam_orientation_blue = np.array([cam_orientation_red[0]+np.pi, cam_orientation_red[1], cam_orientation_red[2]])

if debug:
    cam_position = np.zeros((3))
    cam_orientation = np.zeros((3))
else:
    if team_color == "RED":
        cam_position = cam_position_red
        cam_orientation = cam_orientation_red
    else:
        cam_position = cam_position_blue
        cam_orientation = cam_orientation_blue
def globalXYZToCamXYZ(x, y, z):
    #平移
    x1, y1, z1 = np.array([x, y, z]) - cam_position
    #yaw
    yaw = cam_orientation[0]
    x2 = x1 * np.cos(-yaw) + y1 * np.sin(-yaw)
    y2 = y1 * np.cos(-yaw) - x1 * np.sin(-yaw)
    z2 = z1
    #pitch
    pitch = cam_orientation[1]
    x3 = x2 * np.cos(-pitch) - z2 * np.sin(-pitch)
    y3 = y2
    z3 = z2 * np.cos(-pitch) + x2 * np.sin(-pitch)
    #roll
    roll = cam_orientation[2]
    x_result = x3
    y_result = y3 * np.cos(-roll) - z3 * np.sin(-roll)
    z_result = z3 * np.cos(-roll) + y3 * np.sin(-roll)
    result = np.array([x_result, y_result, z_result])
    return result

def camXYZToGlobalXYZ(x, y, z):
    #roll
    roll = cam_orientation[2]
    x1 = x
    y1 = y * np.cos(roll) - z * np.sin(roll)
    z1 = z * np.cos(roll) + y * np.sin(roll)
    #pitch
    pitch = cam_orientation[1]
    x2 = x1 * np.cos(pitch) - z1 * np.sin(pitch)
    y2 = y1
    z2 = z1 * np.cos(pitch) + x1 * np.sin(pitch)
    #yaw
    yaw = cam_orientation[0]
    x3 = x2 * np.cos(yaw) + y2 * np.sin(yaw)
    y3 = y2 * np.cos(yaw) - x2 * np.sin(yaw)
    z3 = z2
    #平移
    result = np.array([x3, y3, z3]) + cam_position
    return result

if debug:
    """ map_x = np.zeros((1024, 1024), dtype=np.float32)
    map_y = np.zeros((1024, 1024), dtype=np.float32)
    print("!!!")
    for y in range(1024):
        for x in range(1024):
            x_origin, y_origin = radToPix(*newPixToRad(x, y))
            map_x[y, x] = x_origin
            map_y[y, x] = y_origin
        print(x_origin, y_origin)
    np.save("map_x", map_x)
    np.save("map_y", map_y) """
    map_x = np.load("map_x.npy")
    map_y = np.load("map_y.npy")

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

        points_in_range = livoxInterface.get_points_in_range(pixToRad(*rand_posi), 0.03, 0.03)
        print(np.median(points_in_range[:,2]))

    def debug_yolo_to_distance(img1, img2, livoxInterface, boxes):
        for box in boxes:
            center = box.xywh[0,:2]
            half_w = box.xywh[0,2]/2
            half_h = box.xywh[0,3]/2
            pts = np.array([ # ← ↓ → ↑
                [center[0]-half_w, center[1]],
                [center[0], center[1]+half_h],
                [center[0]+half_w, center[1]],
                [center[0], center[1]-half_h]
            ], dtype=np.float64)
            cv2.polylines(img1, [pts.reshape((-1, 1, 2)).astype(np.int32)], isClosed=False, color=(0, 255, 0), thickness=5)

            rads = np.array([pixToRad(*pt) for pt in pts], dtype=np.float64)
            
            rad_center = np.average(rads, axis=0)
            rad_half_w = (rads[2,0] - rads[0,0]) / 2
            rad_half_h = (rads[3,1] - rads[1,1]) / 2
            new_rads = np.array([ # ← ↓ → ↑
                [rad_center[0]-rad_half_w, rad_center[1]],
                [rad_center[0], rad_center[1]-rad_half_h],
                [rad_center[0]+rad_half_w, rad_center[1]],
                [rad_center[0], rad_center[1]+rad_half_h]
            ], dtype=np.float64)
            new_pts = np.array([radToNewPix(*rad) for rad in new_rads], dtype=np.int32)
            cv2.polylines(img2, [new_pts.reshape((-1, 1, 2)).astype(np.int32)], isClosed=False, color=(0, 255, 0), thickness=3)

            points_in_range = livoxInterface.get_points_in_range(rad_center, rad_half_w, rad_half_h)
            target_distance = np.median(points_in_range[:,2])
            #print(rads, rad_center, rad_half_w, rad_half_h, target_distance)
            cv2.putText(img2, f"dis: {target_distance:.2f} mm", radToNewPix(*rads[0]).astype(np.int32), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1, 
                        [0, 255, 0], 
                        1, 
                        cv2.LINE_AA)
            
            target_position_cam = radToCamXYZ(rad_center[0], rad_center[1], target_distance)
            target_position_global = camXYZToGlobalXYZ(*target_position_cam)
            cv2.putText(img2, f"x:[{target_position_global[0]:.2f}], y:[{target_position_global[1]:.2f}], z:[{target_position_global[2]:.2f}]", radToNewPix(*rads[1]).astype(np.int32), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        1, 
                        [0, 255, 0], 
                        1, 
                        cv2.LINE_AA)
