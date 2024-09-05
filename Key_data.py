from select import select
from PIL import Image
import ardrone
import numpy as np
import sys
import termios
import tty
import cv2
import time

class my_ARDrone(ardrone.ARDrone):
    def set_yaw(self, yaw):
        self.yaw = yaw

    def move_turn_left_left(self):
        drone.move(-self.speed, 0, 0, -self.yaw)

    def move_turn_right_right(self):
        drone.move(self.speed, 0, 0, self.yaw)

    def turn_left(self):
        self.atcmd.pcmd(True, 0, 0, 0, -self.yaw)

    def turn_right(self): 
        self.atcmd.pcmd(True, 0, 0, 0, self.yaw)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key
'''
def baza_cmd(cmd):
    if cmd == '1':
        drone.takeoff()
    elif cmd == '2':
        drone.land()
    elif cmd == "k":
        drone.hover()
    elif cmd == 'i':
        drone.move_forward()
    elif cmd == ',':
        drone.move_backward()
    elif cmd == "j":
        drone.turn_left()
    elif cmd == 'l':
        drone.turn_right()
    elif cmd == 'q':
        drone.move_up()
    elif cmd == 'z':
        drone.move_down()
    elif cmd == 'u':
        drone.move_turn_left_left()
    elif cmd == 'o':
        drone.move_turn_right_right()
    elif cmd == 'J':
        drone.move_left()
    elif cmd == 'L':
        drone.move_right()
    elif cmd == 'a':
        drone.reset()




cmd = ['1', '2', 'q', 'z', 'i', ',', 'j', 'l', 'J', 'L', 'u', 'o', 'k', 'a']
'''


if __name__ == "__main__":

    settings = saveTerminalSettings()
    key_timeout = 0.5

    drone = my_ARDrone()
    
    drone.set_speed(0.05)
    drone.set_yaw(0.35)

    box_cmds = {
        '1': drone.takeoff,
        '2': drone.land,
        'a': drone.reset,
        'q': drone.move_up,
        'z': drone.move_down,
        'i': drone.move_forward,
        ',': drone.move_backward,
        'k': drone.hover,
        'j': drone.turn_left,
        'l': drone.turn_right,
        'u': drone.move_turn_left_left,
        'o': drone.move_turn_right_right,
        'J': drone.move_left,
        'L': drone.move_right
    }


    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter('IMG_box//Video_from_drone_101.mp4', fourcc, 24.0, (640, 360))
    try:
        while True:
            tic = time.time()
            key = getKey(settings, key_timeout)

            if key in box_cmds.keys():
                box_cmds[key]()
            
            if key == "\x03":
                break

            drone.set_cam(1)
            img = np.array(drone.image)
            out.write(img)
            
            cv2.imwrite("stream.jpg", img)

            toc = time.time()
            sleepTime = 0.04 - (toc - tic)
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                # print("warning")
                pass

    except KeyboardInterrupt as e:
        print(e)
        out.release()
    finally:
        restoreTerminalSettings(settings)
        drone.halt()
        out.release()