cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ORB_SLAM3_ROOT_DIR "/home/radxa/ORB_SLAM3") # SET YOUR PATH

set(CMAKE_INSTALL_RPATH "${ORB_SLAM3_ROOT_DIR}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# Eigen3
find_package(Eigen3 3.1.0 REQUIRED)

# OpenCV
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
  endif()
endif()

# Pangolin
find_package(Pangolin REQUIRED)

# ORB_SLAM3
set(ORB_SLAM3_LIBRARY "${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so")
set(DBOW2_LIBRARY "${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so")
set(G2O_LIBRARY "${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib/libg2o.so")

if(NOT EXISTS ${ORB_SLAM3_LIBRARY})
  message(FATAL_ERROR "ORB_SLAM3 library not found at ${ORB_SLAM3_LIBRARY}")
endif()

if(NOT EXISTS ${DBOW2_LIBRARY})
  message(FATAL_ERROR "DBoW2 library not found at ${DBOW2_LIBRARY}")
endif()

if(NOT EXISTS ${G2O_LIBRARY})
  message(FATAL_ERROR "g2o library not found at ${G2O_LIBRARY}")
endif()

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/core
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/types
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/stuff
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

set(LIBS
  ${ORB_SLAM3_LIBRARY}
  ${DBOW2_LIBRARY}
  ${G2O_LIBRARY}
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  GL
  GLU
)

add_executable(mono_node
  src/mono_node.cpp
  src/common.cpp
)
ament_target_dependencies(mono_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  message_filters
  std_msgs
  visualization_msgs
)
target_link_libraries(mono_node
  ${LIBS}
)

install(PROGRAMS
  scripts/rotate.py
  scripts/new_pose_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  mono_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
