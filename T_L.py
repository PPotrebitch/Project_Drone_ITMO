import ardrone
import time

drone = ardrone.ARDrone()

if __name__ == '__main__':
    drone.takeoff()
    time.sleep(8)
    drone.land()