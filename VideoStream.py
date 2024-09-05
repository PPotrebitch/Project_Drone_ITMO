from PIL import Image 
import ardrone
import time
import cv2
import numpy as np

drone = ardrone.ARDrone()
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('IMG_box//Video_from_drone.mp4', fourcc, 24.0, (640, 360))

try:
    while True:
        drone.set_cam(1)
        img = np.array(drone.image)
        # print(img.shape)
        # print(img)

        cv2.imwrite()
        out.write(img)
        time.sleep(0.04)
except:
    out.release()