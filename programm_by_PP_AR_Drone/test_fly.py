import ardrone
import time

drone = ardrone.ARDrone()

if __name__ == '__main__':
    drone.set_speed(0.05)
    drone.takeoff()
    time.sleep(5)
    drone.hover()
    drone.move_forward()
    time.sleep(2)
    drone.hover()
    drone.move_backward()
    time.sleep(2)
    drone.hover()
    drone.move_left()
    time.sleep(2)
    drone.hover()
    drone.move_right()
    time.sleep(2)
    drone.land()