file(GLOB GUI_SRCS *.cpp)
file(GLOB GUI_HDRS *.h)
include_directories(${CMAKE_SOURCE_DIR}/GUI/src)
add_library(GUI ${GUI_SRCS} ${GUI_HDRS})