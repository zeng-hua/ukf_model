CMAKE_MINIMUM_REQUIRED(VERSION 3.7 FATAL_ERROR)
PROJECT(oneDOF)

# Disable dynamic memory allocation in Eigen
ADD_DEFINITIONS(-DEIGEN_NO_MALLOC -DUKF_DOUBLE_PRECISION)

INCLUDE_DIRECTORIES(../../include ${eigen_dir})
#INCLUDE_DIRECTORIES(${eigen_dir})
cmake_policy(SET CMP0042 NEW)

#ADD_LIBRARY(oneDOF SHARED spring_damping_system.cpp)
#ADD_LIBRARY(oneDOF SHARED test.cpp)
#SET_TARGET_PROPERTIES(oneDOF PROPERTIES OUTPUT_NAME oneDOF)

#target_link_libraries(oneDOF "${TORCH_LIBRARIES}")

#set_property(TARGET oneDOF PROPERTY CXX_STANDARD 14)

#add_executable(TEST spring_damping_system.cpp)
#
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_STANDARD 11)

add_executable(TEST test.cpp)
#add_executable(TEST test2.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

target_link_libraries(TEST "${TORCH_LIBRARIES}")

#set_property(TARGET TEST PROPERTY CXX_STANDARD 11)
