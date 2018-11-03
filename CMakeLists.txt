cmake_minimum_required(VERSION 3.5)
project(recorder_visualiser)

# enable C++11 standard (current directory scope)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # prevent fallback

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  sensor_msgs 
  message_filters 
  image_transport 
  cv_bridge
  image_geometry
  rosbag
  tf2_ros
  tf2_eigen

  dlib_ros

)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
	# INCLUDE_DIRS
	# LIBRARIES
	# CATKIN_DEPENDS
	# DEPENDS DBoW2 OpenCV EIGEN3 # this is just for checking purposes
)

add_executable(recorder_visualiser src/main.cpp)


target_include_directories(recorder_visualiser
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(recorder_visualiser ${catkin_LIBRARIES})