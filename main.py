import cv2

import os, sys
sys.path.append(os.path.dirname(__file__))
from LivoxCtypesInterface import LivoxInterface
from Camera import Camera

def main():
    livoxInterface = LivoxInterface()
    """ livoxInterface.pyif_Init()
    livoxInterface.test()
    livoxInterface.pyif_Uninit() """

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
