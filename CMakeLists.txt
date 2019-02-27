cmake_minimum_required(VERSION 2.9.0)

project(geos-perception)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_SOURCE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

#add_subdirectory(${CMAKE_SOURCE_DIR}/GUI/)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CUDA REQUIRED)
find_package(efusion REQUIRED)

find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(ZLIB REQUIRED)
find_package(OpenNI2 REQUIRED)
find_package(efusion REQUIRED)
find_package(SuiteSparse REQUIRED)

file(GLOB SRCS ${CMAKE_SOURCE_PATH}/src/*.cpp)
file(GLOB HDRS *.h)

set(SOURCES
    ${CMAKE_SOURCE_PATH}/src/pcore.cpp
    ${CMAKE_SOURCE_PATH}/src/EFusionWrapper.cpp
    ${CMAKE_SOURCE_PATH}/src/utils/datareader.cpp
    ${CMAKE_SOURCE_PATH}/src/RawLogReader.cpp
    ${SRCS}
)

# Additional Headers
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EFUSION_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}
    ${OPENNI2_INCLUDE_DIR}
    ${HDRS}
#    ${CMAKE_SOURCE_PATH}/GUI/inc
)

#link_directories(
#    ${CMAKE_SOURCE_PATH}/GUI/src
#)

# Executable
add_executable(
    pcore 
    ${SOURCES}
    ${SRCS}
    ${HDRS}
    )

target_link_libraries(
    pcore
    ${OpenCV_LIBRARIES}
    ${EFUSION_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${EXTRA_WINDOWS_LIBS}
    ${ZLIB_LIBRARY}
    ${CUDA_LIBRARIES}
    ${EXTRA_LIBS}
    ${OPENNI2_LIBRARY}
    ${SUITESPARSE_LIBRARIES}
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
#    GUI
)

#file(GLOB GUI_SRCS *.cpp)
#
#include_directories(${CMAKE_SOURCE_PATH}/GUI/)
#link_directories(${CMAKE_SOURCE_PATH}/GUI/)
#
#set(SOURCES ${GUI_SRCS})
#add_library(GUI ${SOURCES})
#add_executable(geos-perception ${SOURCES})
#target_link_libraries(geos-perception GUI)