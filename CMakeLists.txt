cmake_minimum_required(VERSION 2.9.0)

project(geos-perception)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

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

set(SOURCES
    ${CMAKE_MODULE_PATH}/src/pcore.cpp
    ${CMAKE_MODULE_PATH}/src/EFusionWrapper.cpp
    ${CMAKE_MODULE_PATH}/src/utils/datareader.cpp
    ${CMAKE_MODULE_PATH}/src/utils/GUI/RawLogReader.cpp
)

# Additional Headers
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EFUSION_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS}
    ${OPENNI2_INCLUDE_DIR}
)

# Executable
add_executable(
    pcore 
    ${SOURCES}
    )

target_link_libraries(
    pcore
    ${OpenCV_LIBRARIES}
    ${EFUSION_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${EXTRA_WINDOWS_LIBS}
    ${ZLIB_LIBRARY}
    ${Pangolin_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${EXTRA_LIBS}
    ${EFUSION_LIBRARY}
    ${OPENNI2_LIBRARY}
    ${SUITESPARSE_LIBRARIES}
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
)