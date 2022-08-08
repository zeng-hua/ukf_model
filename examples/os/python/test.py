import ctypes
from ctypes import *
import numpy as np
import matplotlib.pyplot as plt
import pylab
from rk import RK4

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
    return t, y

if __name__=="__main__":
    mem = create_string_buffer(1024*1024*50)
    ll = ctypes.cdll.LoadLibrary

    lib = ll(r'../../../cmake-build-debug/examples/os/libonedof.so')

    lib.ukf_init(c_double(0.0), c_double(1.0))

    x = []
    t = np.arange(0,100,0.1)
    for ii in range(len(t)):
        # lib.ukf_sensor_set_Displacementmeter(c_double(0.01))
        lib.ukf_iterate(c_double(1))
        # print(ii)
        x.append(float(lib.ukf_get_v()))

    plt.plot(t,x)
    plt.show()
    # print(t)


