cmake_minimum_required(VERSION 2.8.3)
project(tracker_kcf)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -pedantic -O2 -Wno-long-long -fno-omit-frame-pointer")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation 
)

find_package(OpenCV REQUIRED)

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  tracker_result.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hps_camera
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories ( ${OpenCV_LIB_DIR} )
MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")

include_directories ( ${OpenCV_INCLUDE_DIRS} )
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR}/)
include_directories(${CMAKE_SOURCE_DIR}/src/)

FILE(GLOB_RECURSE sourcefiles "src/*.cpp")

add_executable(tracker_kcf ${sourcefiles} )
target_link_libraries(tracker_kcf ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

