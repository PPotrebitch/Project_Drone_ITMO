from select import select
import math
import ardrone
import numpy as np
import sys
import termios
import tty
import cv2
import time

import signal
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(filename='line_follower.log', level=logging.DEBUG)

H = 360  # высота изобразения
W = 640  # ширина изображения
MODE = 0
CHANGE_STATE = False

def signal_handler(sig, frame):
    global MODE, CHANGE_STATE
    logger.warning("Press Ctrl-Z")
    MODE = int(input("Copter mode:"))
    logging.info(f"Mode: {MODE}")
    CHANGE_STATE = True


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

class my_ARDrone(ardrone.ARDrone):#  Добавляем недостающие команды для управлением БПЛА

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

            logging.info(f"Markers detected using {aruco_type} dictionary")

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

def get_new_points( v, q):  # Получаем координату точки в новой системе координат,
    '''
    где  оX вверх,а oY влево.
    (g, v) - центр aruco, h и w - высота и ширина изображения.
    '''
    global H, W
    y = -q + W/2
    x = -v + H/2
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
    global H, W
    HImage, WImage = H, W
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255,
	cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 75, 28)
    w_3c = np.full_like(thresh, fill_value=(255))
    center = (thresh.shape[1]//2, thresh.shape[0]//2)
    radius = int(min(center) * .7)
    zeros = np.zeros_like(thresh[:,:], dtype='uint8')
    masked = cv2.bitwise_and(thresh, w_3c, mask=zeros)
    contour=cv2.findContours(masked, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contour=contour[0]
    if len(contour)>0:
        masked = masked
    else:
        masked = thresh

    for i in range(9,-1,-1):
        obrezimage = masked[((radius//5)*i+(HImage//2 - radius)):((radius//5)*(i+1)+(HImage//2 - radius)), 0:WImage]
        contours=cv2.findContours(obrezimage, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours=contours[0]
        if len(contours) > 1:
            contours=sorted(contours, key=cv2.contourArea, reverse=True)
            (x,y,w,h)=cv2.boundingRect(contours[0])
            xx=int(x + w//2)
            yy=int(y + h//2)+(HImage//10)*i
            cv2.rectangle(img,(xx,yy), (xx+2, yy+2),(255,0,0), 5)
            # cv2.imwrite("Line_stream.jpg", image)
            XY.append([xx, yy])
    return XY

def get_line_xy(img):  # Функция для следования по линии
    global H, W
    XY = []
    XY = get_line_points(img)
    if XY != [] and len(XY) >= 3:
        X, Y = 0, 0
        for cord_dot in XY:
            if (H//2 - cord_dot[1]) > 0:
                X = cord_dot[0]
                Y = cord_dot[1]
                # logging.info(f'X = {X}, Y = {Y}')
                break


        cv2.circle(img, (int(W//2), int(H//2)), 5, (0, 255, 0), -1)
        cv2.circle(img, (int(W//2), int(H//2 - 50)), 5, (0, 0, 255), -1)
        cv2.circle(img, (int(X), int(Y)), 7, (255, 255, 0), 1)

        line_x = H//2 - Y
        line_y = W//2 - X
    else:
        line_x, line_y = 0, 0

    # cv2.imwrite("Line_stream.jpg", img)
    return line_x, line_y


def cross(img): # алгоритм для распознования крестика для стабилизации дрона 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    frame1 = cv2.adaptiveThreshold(blurred, 255,
	cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 75, 28)
    #w_3c = np.full_like(thresh, fill_value=(255))
    frame1[0][0]=1
    global H, W
    y1=0
    y2=0
    contours=cv2.findContours(frame1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours=contours[0]
    if contours:
        contours=sorted(contours, key=cv2.contourArea, reverse=True)
        (x,y,w,h)=cv2.boundingRect(contours[0])
        cenerx = x+w//2
    x1=x
    x2=x+w
    contours=cv2.findContours(frame1[:,x1:x1+3], cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours=contours[0]
    if contours:
        contours=sorted(contours, key=cv2.contourArea, reverse=True)
        (x,y,w,h)=cv2.boundingRect(contours[0])
        y1 = y+h//2
    contours=cv2.findContours(frame1[:,x2-3:x2], cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours=contours[0]
    if contours:
        contours=sorted(contours, key=cv2.contourArea, reverse=True)
        (x,y,w,h)=cv2.boundingRect(contours[0])
        y2 = y+h//2
    cenery = (y1+y2)//2
    if cenerx==0 and cenery == 0:
        cenerx = W//2
        cenery = H//2
    return cenerx,cenery

def get_cross_xy(img): #  Функция для следования за крестиком
    global H, W
    cx, cy = cross(img)
    if cx is not None and cy is not None:
        cv2.circle(img, (int(W//2), int(H//2)), 5, (0, 0,  255), -1)
        cv2.circle(img, (int(W//2), int(H//2 - 50)), 5, (0, 255, 0), -1)
        cv2.circle(img, (int(cx), int(cy)), 7, (255, 0, 0), -1)
        cv2.imwrite("Cross_stream.jpg", img)

        line_x = H//2 - cy
        line_y = W//2 - cx
        #logging.info(f'x == {cx}. y == {cy}; line_x = {line_x}, line_y = {line_y}',end=";")
    else:
        line_x, line_y = 0, 0
    return line_x, line_y



def control_keyboard(drone: my_ARDrone, settings, key_timeout): # Запуск алгоритма обраьотки сигнала с клавиаиуры 

    logging.info("[keyboard] Init")

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

    logging.info("[keyboard] Start")
    while not CHANGE_STATE:
        tic = time.time()

        key = getKey(settings, key_timeout)
        if key in box_cmds.keys():
            logging.info(f"key {key}")
            box_cmds[key]()

        toc = time.time()
        sleepTime = 0.8 - (toc - tic)
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            logging.info("[keyboard] warning")

    logging.info("[keyboard] Exit")

def control_line(drone: my_ARDrone):  # Запуск алгоритма по линии  

    logging.info("[line] Init")

    drone.set_cam(1)

    # Устанавливаем линейную и угловую скорость
    drone.set_speed(0.08)
    drone.set_yaw(0.35)

    # Подбираем коэффициенты для ПИД
    H = 1.0
    PID_line_x = PID_controller(k_p=0.0, k_i=0.0, k_d=0.0, h=H)
    PID_line_y = PID_controller(k_p=0.001, k_i=0.0, k_d=0.0, h=H)
    PID_yaw = PID_controller(k_p=0.0, k_i=0.0, k_d=0.0, h=H)

    # Desired line point
    cx = 0
    cy = 0
    cx_last = 0
    cy_last = 0
    etheta_last = 0
    etheta = 0
    # Control inputs
    vx = 0
    vy = 0
    wz = 0
    
    logging.info("[line] Start")
    while not CHANGE_STATE:
        tic = time.time()
        img = np.array(drone.image)
        cv2.imwrite("Line_stream.jpg", img)

        if img is not None:
            etheta_last = etheta
            cx, cy = get_line_xy(img)
            cx_last, cy_last = cx, cy
            etheta = -math.atan2(cy, cx)

            # Используем ПД-регулятор
            # vx = PID_line_x.updatePD(e_now=cx, e_last=cx_last)
            vy = PID_line_y.updatePD(e_now=cy, e_last=cy_last)
            # wz = PID_yaw.updatePD(e_now=etheta, e_last=etheta_last)

            # logging.info(f"Center {cx}, {cy}")
            # logging.info(f"Control {vy}")
            
            
            if vx > 1.0:
                vx = 1.0
            elif vx < -1.0:
                vx = -1.0

            if vy > 1.0:
                vy = 1.0
            elif vy < -1.0:
                vy = -1.0

            if wz > 1.0:
                wz = 1.0
            elif wz < -1.0:
                wz = -1.0


            drone.move_xyzw(vx, vy, 0, wz)
        else:
            vx = 0
            vy = 0
            wz = 0
            drone.hover()

        # logging.info(f' vx = {vx}, vy = {vy}, wz = {wz}')
        toc = time.time()
        sleepTime = H - (toc - tic)
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            logging.warning("[line] warning")
        
    logging.info("[line] Exit")


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
        sleepTime = 1.0/30 - (toc - tic)
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            logging.info("warning")

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
    signal.signal(signal.SIGTSTP, signal_handler)
    logger.info(f"Start the loop with mode {MODE}")

    try:

        while True:

            tic = time.time()
            key = getKey(settings, key_timeout)

            img = np.array(drone.image)
            cv2.imwrite("Line_stream.jpg", img)

            toc = time.time()
            sleepTime = 1/10 - (toc - tic)
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                logging.warning("[main] warning")

    except KeyboardInterrupt as e:
        logging.error(e)
    finally:
        restoreTerminalSettings(settings)
        drone.halt()