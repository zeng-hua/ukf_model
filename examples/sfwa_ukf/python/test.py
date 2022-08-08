import ctypes
ll = ctypes.cdll.LoadLibrary

lib = ll(r'../../../cmake-build-debug/examples/sfwa_ukf/libcukf.so')

print(lib.UKF_MODEL_X8)