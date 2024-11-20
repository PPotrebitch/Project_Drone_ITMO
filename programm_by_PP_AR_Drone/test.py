from select import select
import math
# import ardrone
import numpy as np
import sys
import termios
import tty
# import cv2
import time

import signal
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(filename='line_follower.log', encoding='utf-8', level=logging.DEBUG)

H = 360  # высота изобразения
W = 640  # ширина изображения

def signal_handler(sig, frame):
    logger.warning("Press Ctrl-Z")


if __name__ == "__main__":

    signal.signal(signal.SIGTSTP, signal_handler)

    while True:
        time.sleep(1)
        print("ping")