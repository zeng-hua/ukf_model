CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4 FATAL_ERROR)
PROJECT(sfwa_ukf)

# Disable dynamic memory allocation in Eigen
ADD_DEFINITIONS(-DEIGEN_NO_MALLOC -DUKF_DOUBLE_PRECISION)

INCLUDE_DIRECTORIES(../../include ${eigen_dir})

cmake_policy(SET CMP0042 NEW)

ADD_LIBRARY(sfwa_ukf SHARED cukf.cpp)
SET_TARGET_PROPERTIES(sfwa_ukf PROPERTIES OUTPUT_NAME cukf)

#add_executable(sfwa_ukf test.cpp)