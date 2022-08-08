CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4 FATAL_ERROR)
PROJECT(onedof)

# Disable dynamic memory allocation in Eigen
ADD_DEFINITIONS(-DEIGEN_NO_MALLOC -DUKF_SINGLE_PRECISION)

INCLUDE_DIRECTORIES(../../include ${eigen_dir})

cmake_policy(SET CMP0042 NEW)

ADD_LIBRARY(onedof SHARED onedof_v1.cpp)
#ADD_LIBRARY(ahrs SHARED spring_damping_system.cpp)
SET_TARGET_PROPERTIES(onedof PROPERTIES OUTPUT_NAME onedof)