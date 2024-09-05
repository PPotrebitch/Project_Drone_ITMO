import time
import matplotlib.pyplot as plt
import numpy as np
from math import pi

class PID_controller():
    def __init__(self, k_p, k_i, k_d, h):
        self.integral = 0
        self.h = h

        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

    def Proportional(self, e_now):
        return e_now

    def Integral(self, e_now, e_last):
        self.integral =  self.integral + (e_now + e_last)*(self.h/2)
        return self.integral

    def Differential(self, e_now, e_last):
        return (e_now - e_last)/self.h

    def PID_result(self, e_now, e_last):
        self.e_now = e_now
        self.e_last = e_last
        u = self.k_p *self.Proportional(e_now) + self.k_i * self.Integral(e_now, e_last) + self.k_d * self.Differential(e_now, e_last)
        return u


'''
if __name__ == "__main__":

    best_theta = 90
    pos_now = 0
    time_now = 0
    integral = 0
    e_now = 0
    e_last = 0

    frequency = 31 # Hz
    h = 1.0/frequency
    box = []
    Timer = []

    counter = 0

    try:
        time_start = time.time()
        start_pos = 0
        u = 0
        PID = PID_controller(k_p=0.4, k_i=1, k_d=0, h=h)
        while time_now < 20:
            t1 = time.time()
            time_now = time.time() - time_start
            pos = start_pos + u
            pos_now = pos - start_pos
            e_last = e_now
            e_now = (best_theta - pos_now)

            u = PID.PID_result(e_now=e_now, e_last=e_last)

            Timer.append(time_now)
            box.append(pos_now)

            counter += 1
            t2 = time.time()
            dt = h - (t2 - t1)
            if dt > 0:
                time.sleep(dt)
            else:
                print("!error!")

        time_iteration = (time.time() - time_start)/counter
        print(f"Time for each iteration: {time_iteration} | frequency: {1/time_iteration}")
        # # plt.style.use('https://github.com/dhaitz/matplotlib-stylesheets/raw/master/pitayasmoothie-dark.mplstyle')
        # # plt.style.context('dark_background'):
        plt.style.use("fivethirtyeight")
        plt.axhline(y = 90, color = 'r', linestyle = '--')
        plt.plot(Timer, box)
        plt.xlabel('time, s')
        plt.ylabel('pos')
        plt.grid(True, linewidth=1.5)
        plt.show()
    except Exception as e:
        raise e
    finally:
        pass
'''