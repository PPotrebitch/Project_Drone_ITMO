import ardrone
import time
from PIL import Image

drone = ardrone.ARDrone()
while True:
    drone.set_cam(1)
    img = drone.image
    img.save('IMG_box/img_1.png')
    print(drone.image)
    time.sleep(0.25)
