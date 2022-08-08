import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from rk import RK4
import sys

def wgn(x, snr):
    snr = 10**(snr/10.0)
    xpower = np.sum(x**2)/len(x)
    npower = xpower / snr
    return np.random.randn(len(x)) * np.sqrt(npower)

def sim():
    # Coefficient of Friction
    m = 1

    # Spring Constant
    c = 0.5

    # Mass
    k = 10

    ydot = lambda t, x, y: -(c*y+k*x) / m
    xdot = lambda t, x, y: y

    lv = RK4(xdot, ydot)
    t, y = lv.solve([0, 1], .01, 20)

    data = pd.DataFrame()
    data['t'] = t
    data['x'] = y[0] + wgn(np.array(y[0]), 6)
    data['y'] = y[1] + wgn(np.array(y[1]), 6)
    data['xr'] = y[0]
    data['yr'] = y[1]

    data.to_csv('/home/zenghua/code/ukf/examples/test/python/onedof.csv', header=None, index=None)

    return t, y

if __name__=="__main__":
    sys.path.append('/home/zenghua/code/ukf/examples/test/python/')

    sim()
    sys_data = pd.read_csv('onedof.csv', header=None)
    cpp_data = pd.read_csv('onedof_cpp.csv', header=None)

    plt.plot(sys_data[0], sys_data[1],'k--')
    plt.plot(sys_data[0], sys_data[3],'r',linewidth=2)
    # plt.plot(sys_data[0], sys_data[2]+1)
    # plt.plot(cpp_data[0], cpp_data[2],'c',linewidth=2)
    # plt.legend(['noise v', 'real v', 'ukf v'])
    plt.legend(['noise v', 'real v'])
    plt.ylabel('v(m)')
    plt.xlabel('t(s)')


    plt.show()

    # print(sys_data.columns)


