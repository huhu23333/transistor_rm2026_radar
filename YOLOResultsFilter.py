import numpy as np

# 定义IOSA计算函数（相交面积与较小框面积的比率）
def calculate_iosa(box1, box2):
    """
    计算两个框的相交面积与较小框面积的比率
    
    参数:
    box1: 第一个框的坐标 [x1, y1, x2, y2]
    box2: 第二个框的坐标 [x1, y1, x2, y2]
    
    返回:
    两个框的IOSA值
    """
    # 计算交集区域的坐标
    x1_inter = max(box1[0], box2[0])
    y1_inter = max(box1[1], box2[1])
    x2_inter = min(box1[2], box2[2])
    y2_inter = min(box1[3], box2[3])
    
    # 计算交集面积
    inter_area = max(0, x2_inter - x1_inter) * max(0, y2_inter - y1_inter)
    
    # 计算各自面积
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    
    # 获取较小框的面积
    min_area = min(area1, area2)
    
    # 避免除以零
    if min_area == 0:
        return 0
    
    # 计算IOSA
    iosa = inter_area / min_area
    return iosa

# 定义去除重叠框的函数（使用IOSA）
def remove_overlapping_boxes(boxes, iosa_threshold=0.5):
    """
    去除重叠IOSA大于阈值的框，保留面积较大的框
    
    参数:
    boxes: YOLO识别结果的boxes对象
    iosa_threshold: IOSA阈值，大于此值的重叠框将被去除
    
    返回:
    过滤后的boxes索引列表
    """
    if hasattr(boxes, 'xyxy'):
        box_coords = boxes.xyxy.cpu().numpy()
    else:
        box_coords = boxes.cpu().numpy()
    
    # 计算每个框的面积
    areas = (box_coords[:, 2] - box_coords[:, 0]) * (box_coords[:, 3] - box_coords[:, 1])
    
    # 按面积从大到小排序
    sorted_indices = np.argsort(areas)[::-1]
    
    # 初始化保留的索引列表
    keep_indices = []
    remaining_indices = sorted_indices.tolist()
    
    while len(remaining_indices) > 0:
        current_idx = remaining_indices[0]
        keep_indices.append(current_idx)
        
        if len(remaining_indices) == 1:
            break
            
        current_box = box_coords[current_idx]
        iosas = []
        
        for i in range(1, len(remaining_indices)):
            other_idx = remaining_indices[i]
            other_box = box_coords[other_idx]
            iosa = calculate_iosa(current_box, other_box)
            iosas.append(iosa)
        
        iosas = np.array(iosas)
        # 保留IOSA小于阈值的框
        low_overlap_indices = np.where(iosas < iosa_threshold)[0]
        remaining_indices = [remaining_indices[i+1] for i in low_overlap_indices]
    
    return keep_indices
