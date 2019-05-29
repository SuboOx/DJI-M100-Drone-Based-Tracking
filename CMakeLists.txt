cmake_minimum_required(VERSION 2.8.3)
project(track_pkg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -pedantic -O2 -Wno-long-long -fno-omit-frame-pointer")

catkin_package()
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  dji_sdk
)

find_package(DJIOSDK REQUIRED)
find_package(OpenCV REQUIRED)

#win32 is not supported

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

link_directories ( ${OpenCV_LIB_DIR} )
MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")

include_directories ( ${OpenCV_INCLUDE_DIRS} )
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR}/)
include_directories(${CMAKE_SOURCE_DIR}/src/)

FILE(GLOB KCF_SRC_FILE
	src/visual_tracker.cpp
        src/fhog.cpp
        src/kcftracker.cpp)

#add_executable( kcf_node ${KCF_SRC_FILE})
#link_directories(/usr/local/lib /usr/lib/arm-linux-gnueabihf /usr/lib)
#target_link_libraries( kcf_node dcam ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
#Wtf is dcam, without it, the whole thing is not working.
#It turns out to be a dynamic linkage 

add_executable(flight_control
	${KCF_SRC_FILE}
	src/mission_planner.cpp
        src/flight_control.cpp
	src/gimbal.cpp
)
link_directories(/usr/local/lib /usr/lib/arm-linux-gnueabihf /usr/lib)
target_link_libraries(flight_control
	dcam
	${OpenCV_LIBS}
	${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(flight_control dji_sdk_generate_messages_cpp)

