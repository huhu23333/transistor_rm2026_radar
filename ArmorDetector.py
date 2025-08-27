import cv2
import numpy as np
import math

class LightBar:
    def __init__(self, center, LW, angle):
        self.center = center
        self.length = LW[0]
        self.width = LW[1]
        self.angle = angle

    def get_rect(self):
        return (self.center,(self.length,self.width),self.angle)
    
    def get_box(self):
        return cv2.boxPoints(self.get_rect())
    
    def draw(self, img):
        box = self.get_box()
        box = np.intp(box)
        cv2.drawContours(img, [box], 0, (0, 255, 255), 2)

    def length_direction(self):
        rad = self.angle*math.pi/180
        return (math.cos(rad), math.sin(rad))
    
    def width_direction(self):
        rad = self.angle*math.pi/180
        return (-math.sin(rad), math.cos(rad))
    
    def armer_vertexes(self, dcenter):
        a = 2.2
        length_direction = self.length_direction()
        width_direction = self.width_direction()
        length_direction_vector = [self.length*length_direction[0]/2*a, self.length*length_direction[1]/2*a]
        half_width_to_dcenter_vector = [self.width*width_direction[0]/2, self.width*width_direction[1]/2]
        if width_direction[0]*dcenter[0]+width_direction[1]*dcenter[1] < 0:
            half_width_to_dcenter_vector = [-half_width_to_dcenter_vector[0], -half_width_to_dcenter_vector[1]]
        return [(self.center[0]-length_direction_vector[0]+half_width_to_dcenter_vector[0], self.center[1]-length_direction_vector[1]+half_width_to_dcenter_vector[1]),
                (self.center[0]+length_direction_vector[0]+half_width_to_dcenter_vector[0], self.center[1]+length_direction_vector[1]+half_width_to_dcenter_vector[1])]
    
    def pair_to(self, another, angle_threshold=15, 
                length_threshold_ratio=0.8, 
                length_direction_threshold_ratio=2.0, 
                distance_ratio_threshold_low=0.5, 
                distance_ratio_threshold_high=5.0, 
                backward=False):
        #big_ratio = 3.67
        #small_ratio = 2.12
        angle_mismatch = abs(self.angle-another.angle)
        angle_match = angle_mismatch<=angle_threshold or abs(angle_mismatch-180)<=angle_threshold
        if not angle_match:
            return False
        length_mismatch = abs(self.length-another.length)/self.length
        length_match = length_mismatch <= length_threshold_ratio
        if not length_match:
            return False
        dcenter = (self.center[0]-another.center[0], self.center[1]-another.center[1])
        length_direction = self.length_direction()
        length_direction_mismatch = abs(dcenter[0]*length_direction[0]+dcenter[1]*length_direction[1])/self.length
        length_direction_match = length_direction_mismatch <= length_direction_threshold_ratio
        if not length_direction_match:
            return False
        width_direction = self.width_direction()
        width_distance = abs(dcenter[0]*width_direction[0]+dcenter[1]*width_direction[1])
        distance_match = distance_ratio_threshold_low <= width_distance/self.length <= distance_ratio_threshold_high
        if not distance_match:
            return False
        if backward:
            return True
        if another.pair_to(self, backward=True):
            return True
        else:
            return False
        

class Armer:
    def __init__(self, lightBar1, lightBar2):
        self.lightBar1 = lightBar1
        self.lightBar2 = lightBar2
        dcenter = (lightBar2.center[0]-lightBar1.center[0], lightBar2.center[1]-lightBar1.center[1])
        points1 = lightBar1.armer_vertexes(dcenter)
        dcenter = (lightBar1.center[0]-lightBar2.center[0], lightBar1.center[1]-lightBar2.center[1])
        points2 = lightBar2.armer_vertexes(dcenter)
        if (points1[1][0]-points1[0][0])*(points2[1][0]-points2[0][0])+(points1[1][1]-points1[0][1])*(points2[1][1]-points2[0][1]) > 0:
            points2 = [points2[1],points2[0]]
        self.box = np.array(points1+points2)
        self.angle_mismatch = abs(lightBar1.angle-lightBar2.angle)

    def draw(self, img):
        self.lightBar1.draw(img)
        self.lightBar2.draw(img)
        box = np.intp(self.box)
        cv2.drawContours(img, [box], 0, (0, 255, 0), 2)


class ArmorDetector:
    def __init__(self):
        return
    
    def detectArmors(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, img_binary = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lightBars = []
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            area = cv2.contourArea(cnt)
            side1, side2 = rect[1]
            length = max(side1, side2)
            width = min(side1, side2)
            if width > 0 and length / width > 3 and length > 5 and area > 20:
                center = rect[0]
                angle = rect[2]
                if length!=side1:
                    angle += 90
                lightBars.append(LightBar(center,(length,width),angle))
        armers = []
        lightBarIndexUsingArmerIndexes = [[] for i in range(len(lightBars))]
        for i in range(len(lightBars)):
            for j in range(i+1,len(lightBars)):
                if lightBars[i].pair_to(lightBars[j]):
                    lightBarIndexUsingArmerIndexes[i].append(len(armers))
                    lightBarIndexUsingArmerIndexes[j].append(len(armers))
                    armers.append(Armer(lightBars[i], lightBars[j]))
        armerIndexesToDel = []
        for armerIndexes in lightBarIndexUsingArmerIndexes:
            if len(armerIndexes) > 1:
                angle_mismatches_to_compare = []
                for armerIndex in armerIndexes:
                    angle_mismatches_to_compare.append(armers[armerIndex].angle_mismatch)
                toDelIndexes = armerIndexes.copy()
                del toDelIndexes[angle_mismatches_to_compare.index(min(angle_mismatches_to_compare))]
                armerIndexesToDel += toDelIndexes
        for delIndex in sorted(set(armerIndexesToDel), reverse=True):
            del armers[delIndex]
        for armer in armers:
            armer.draw(image)