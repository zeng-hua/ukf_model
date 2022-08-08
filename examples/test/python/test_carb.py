import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from rk import RK4 # https://github.com/sbillaudelle/runge-kutta
import sys

def d(t):
    if t >= 5  and t < 10:
        return 10
    else:
        return 0

def sim():
    tao = 10
    D1dot = lambda t, D1, D2: -D1/tao+d(t)
    D2dot = lambda t, D1, D2: (D1-D2)/tao
    lv = RK4(D1dot, D2dot)
    # lv = RK4(D1dot)






    t, y = lv.solve([0, 0], .01, 100)

    dt = [d(it) for it in t]

    data = pd.DataFrame()
    data['t'] = t
    data['D1'] = y[0]
    data['D2'] = y[1]
    data['in'] = dt

    data.to_csv('/home/zenghua/code/ukf/examples/test/python/carb.csv', header=None, index=None)

    plt.plot(t,y[1])
    # plt.plot(t,y[0],t,y[1])
    # plt.plot(t,d(t))
    plt.show()

if __name__=='__main__':
    sim()

