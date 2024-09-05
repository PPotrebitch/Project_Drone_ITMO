from select import select
from math import atan2
import ardrone
import numpy as np
import sys
import termios
import tty
import cv2
import time

import threading
mutex = threading.Lock()



class PID_controller():    # Класс для работы с ПИД-регулятором

    def __init__(self, k_p:float, k_i:float, k_d:float, h: float):
        self.integral = 0
        self.h = h

        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

    def Proportional(self, e_now: float) -> float:
        return e_now

    def Integral(self, e_now:float, e_last: float) -> float:
        self.integral += (e_now + e_last)*(self.h/2)
        return self.integral

    def Differential(self, e_now:float, e_last: float) -> float:
        return (e_now - e_last)/self.h

    # def PID_result(self, e_now, e_last: float):
    #     self.e_now = e_now
    #     self.e_last = e_last
    #     u = self.k_p * self.Proportional(e_now) + self.k_i * self.Integral(e_now, e_last) + self.k_d * self.Differential(e_now, e_last)
    #     return u

    def updateP(self, e_now: float) -> float:
        return self.k_p * e_now

    def updatePI(self, e_now: float, e_last: float) -> float:
        return self.updateP(e_now) + self.k_i * self.Integral(e_now + e_last)

    def updatePD(self, e_now: float, e_last: float) -> float:
        return self.updateP(e_now) + self.k_d * self.Differential(e_now, e_last)

    def updatePID(self, e_now: float, e_last: float) -> float:
        return self.updateP(e_now) + self.k_i * self.Integral(e_now + e_last) + self.k_d * self.Differential(e_now, e_last)

class my_ARDrone(ardrone.ARDrone):  #  Добавляем недостающие команды для управлением БПЛА

    def set_yaw(self, yaw):
        self.yaw = yaw

    def move_turn_left_left(self):
        self.move(-self.speed, 0, 0, -self.yaw)

    def move_turn_right_right(self):
        self.move(self.speed, 0, 0, self.yaw)

    def turn_left(self):
        self.atcmd.pcmd(True, 0, 0, 0, -self.yaw)

    def turn_right(self):
        self.atcmd.pcmd(True, 0, 0, 0, self.yaw)

    def move_xyzw(self, x, y, z, yaw):
        self.move(-y, -x, z, yaw)

# Словарь для распознования рамера Aruco
ARUCO_DICT = {
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# функция detect_markers  служит для работы с Aruco
def detect_markers(image):

    aruco_type_list = []

    for aruco_type, dictionary_id in ARUCO_DICT.items():

        arucoDict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        arucoParams = cv2.aruco.DetectorParameters()

        corners, ids, _ = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

        if len(corners) > 0:

            aruco_type_list.append(aruco_type)

            print(f"Markers detected using {aruco_type} dictionary")

            for markerCorner, markerId in zip(corners, ids.flatten()):
                corners_aruco = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_aruco

                cv2.polylines(image, [markerCorner.astype(int)], True, (0, 255, 0), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2)
                cY = int((topLeft[1] + bottomRight[1]) / 2)

                # cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)
                return cX, cY, ids
        else:
            return None, None, None
    # return aruco_type_list


    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()

    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)



    if len(corners) > 0:
        for i in range(0, len(ids)):

            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.025, matrix_coefficients, distortion_coefficients)

            cv2.aruco.drawDetectedMarkers(frame, corners)

            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)


    # print(ids)
    return frame

# функции restoreTerminalSettings, saveTerminalSettings и getKey нужны для обработки сигнала с клавиатуры

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

def get_new_points( v, q, h, w):  # Получаем координату точки в новой системе координат,
    '''
    где  оX вверх,а oY влево.
    (g, v) - центр aruco, h и w - высота и ширина изображения.
    '''
    y = -q + w/2
    x = -v + h/2
    return x, y

def h_and_w_img(img):  # Функция, которая возвращает размер изображения (высоту и ширину)
    height, width = img.shape[:2]
    return height, width

def get_aruco_center(img):   # Функция для отслеживания Aruco
   q, v, id = detect_markers(img)   # получаем координаты центра Aruco-макера
   h, w = h_and_w_img(img)
   if np.all(id is not None):
       cx, cy = get_new_points(v, q, h, w)
       return cx, cy # возващаем положение центра Aruco в новой системе координат
   else:
       return None, None

def get_line_points(img):  # Алгоритм для определения линии на изображения
    XY=[]
    HImage, WImage = h_and_w_img(img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255,
	cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 75, 28)
    # cv2.imshow("1", thresh)
    # print("-")
    for i in range(9,-1,-1):
        obrezimage = thresh[(HImage//10)*i:(HImage//10)*(i+1), 0:WImage]
        contours=cv2.findContours(obrezimage, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours=contours[0]
        #cv2.drawContours(image,contours,0,(255,0,0),5)
        # cv2.imshow("18", obrezimage)
        if len(contours) > 1:
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            xx=int(x + w//2)
            yy=int(y + h//2)+(HImage//10)*i
            # cv2.rectangle(img,(xx,yy), (xx+3, yy+3),(255,0,0), 5)
            # cv2.imwrite("Line_stream.jpg", img)
            XY.append([xx, yy])
    # time.sleep(0.008)
    return XY #,img

def get_line_xy(img):   # Функция для следования по линии 
    XY = []
    XY = get_line_points(img)
    if XY != [] and len(XY) >= 3:
    # print(XY)
        X, Y = XY[-3][0], XY[-3][1]
        # print(XY)
        # print(f'X = {X}, Y = {Y}')

        h, w = h_and_w_img(img)
        cv2.circle(img, (int(w//2), int(h//2)), 5, (0, 255, 0), -1)
        cv2.circle(img, (int(w//2), int(h//2 - 50)), 5, (0, 0, 255), -1)
        cv2.circle(img, (int(X), int(Y)), 5, (255, 0, 0), -1)
        cv2.imwrite("Line_stream.jpg", img)

        # line_x, line_y = get_new_points(X, Y, h, w)

        line_x = h//2 - Y
        line_y = w//2 - X
        # print(f'line_x = {line_x}, line_y = {line_y}')
        # print(X, w//2, Y, h//2, line_x, line_y)
    else:
        line_x, line_y = 0, 0

    return line_x, line_y


def control_keyboard(drone: my_ARDrone, settings, key_timeout): # Запуск алгоритма обраьотки сигнала с клавиаиуры 

    print("[keyboard] Init")

    drone.set_cam(1)

    # Устанавливаем линейную и угловую скорость
    drone.set_speed(0.06)
    drone.set_yaw(0.35)

    # Словарь связки символами клавиатуры  с командами управлением дрона
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

    print("[keyboard] Start")
    while not CHANGE_STATE:
        tic = time.time()

        key = getKey(settings, key_timeout)
        if key in box_cmds.keys():
            print("key", key)
            box_cmds[key]()

        toc = time.time()
        sleepTime = 0.8 - (toc - tic)
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            print("[keyboard] warning")

    print("[keyboard] Exit")

def control_line(drone: my_ARDrone):  # Запуск алгоритма по линии  

    print("[line] Init")

    drone.set_cam(1)

    # Устанавливаем линейную и угловую скорость
    drone.set_speed(0.06)
    drone.set_yaw(0.35)

    # Подбираем коэффициенты для ПИД
    PID_line_x = PID_controller(k_p=0.0, k_i=0.0, k_d=0.0, h=0.04)
    PID_line_y = PID_controller(k_p=0.0, k_i=0.0, k_d=0.0, h=0.04)
    # PID_line_x = PID_controller(k_p=1.5*10**(-4), k_i=0.0, k_d=0.0, h=0.04)
    # PID_line_y = PID_controller(k_p=1.25*10**(-4), k_i=0.0, k_d=0.0, h=0.04)
    PID_yaw = PID_controller(k_p=0.5, k_i=0.0, k_d=0.0, h=0.04)

    # Desired line point
    cx = 0
    cy = 0

    # Control inputs=
    vx = 0
    vy = 0
    wz = 0

    print("[line] Start")
    while not CHANGE_STATE:
        tic = time.time()

        img = np.array(drone.image)

        if img is not None:
            cx, cy = get_line_xy(img)
            etheta = atan2(cy, cx)
            vx = PID_line_x.updateP(e_now=cx)
            vy = PID_line_y.updateP(e_now=cy)
            wz = PID_yaw.updateP(e_now=etheta)
        else:
            vx = 0
            vy = 0
            wz = 0

        print(f' vx = {vx}, vy = {vy}, wz = {wz}')

        drone.move_xyzw(vx, vy, 0, wz)

        toc = time.time()
        sleepTime = 0.04 - (toc - tic)
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            print("[line] warning")
        
    print("[line] Exit")


def control_aruco(drone: my_ARDrone):  # Запуск алгоритма для Aruco-иакера
    drone.set_cam(1)

    # Устанавливаем линейную и угловую скорость
    drone.set_speed(0.06)
    drone.set_yaw(0.35)


    # Подбираем коэффициенты для ПИД
    PID_aruco_x = PID_controller(k_p=0.00035, k_i=0.0, k_d=0.0, h=0.04) # Подбираем коэффициенты ПИД-регулятопа
    PID_aruco_y = PID_controller(k_p=0.00035, k_i=0.0, k_d=0.0, h=0.04) # Подбираем коэффициенты ПИД-регулятопа

    # Desired line point
    cx = 0
    cy = 0

    # Control inputs
    vx = 0
    vy = 0


    while not CHANGE_STATE:
        tic = time.time()

        img = np.array(drone.image)
        if img is not None:
            cx, cy = get_aruco_center(img)
            vx = PID_aruco_x.updateP(e_now=cx)
            vy = PID_aruco_y.updateP(e_now=cy)
            cv2.imwrite("test.jpg", img)
        else:
            vx = 0
            vy = 0

        drone.move_xyzw(vx, vy, z= 0.0, yaw=0.0)

        toc = time.time()
        sleepTime = 0.04 - (toc - tic)
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            print("warning")

CHANGE_STATE = False

# Словарь связки символами клавиатуры  с запуском нужного алгоритма 
control_keys = {
    "0": control_keyboard,
    "9": control_line,
    "8": control_aruco
}

if __name__ == "__main__":

    settings = saveTerminalSettings()
    key_timeout = 0.5

    drone = my_ARDrone()

    # Start in keyboar control mode
    control_thread = threading.Thread(target=control_keys["0"], args=(drone, settings, key_timeout))
    control_thread.start()

    try:

        while True:
            # print("Ping")

            tic = time.time()
            key = getKey(settings, key_timeout)

            if key == "\x03":
                break

            if key in control_keys:
                # print("Change mode")

                with mutex:
                    CHANGE_STATE = True
                # print("Change state")
                control_thread.join(0.1)

                if key == "0":
                    print("Mode: keyboard")
                    control_thread = threading.Thread(target=control_keys[key], args=(drone, settings, key_timeout))
                elif key == "9":
                    print("Mode: line")
                    control_thread = threading.Thread(target=control_keys[key], args=(drone,))
                elif key == "8":
                    print("Mode: aruco")
                    control_thread = threading.Thread(target=control_keys[key], args=(drone,))

                with mutex:
                    CHANGE_STATE = False
                control_thread.start()

            toc = time.time()
            sleepTime = 0.8 - (toc - tic)
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                print("[main] warning")

    except KeyboardInterrupt as e:
        print(e)
    finally:
        restoreTerminalSettings(settings)
        drone.halt()
