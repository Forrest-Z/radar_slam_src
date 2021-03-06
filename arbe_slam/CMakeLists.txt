cmake_minimum_required(VERSION 3.0.2)
project(arbe_slam)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  CANROS.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(OpenCV 3.3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arbe_slam
#  CATKIN_DEPENDS message_generation roscpp sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

add_executable(feature_extract 
  src/feature_extract.cpp
)
target_link_libraries(feature_extract 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(os_cfar 
  src/os_cfar.cpp
)
target_link_libraries(os_cfar  
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(multipath 
  src/multipath.cpp
)
target_link_libraries(multipath  
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(radar_odometry 
  src/radar_odometry.cpp
)
target_link_libraries(radar_odometry 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# add_executable(radar_mapping 
#   src/radar_mapping.cpp
# )
# target_link_libraries(radar_mapping 
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

add_executable(radar_tracing 
  src/radar_tracing.cpp
  src/ekf.cpp
)
target_link_libraries(radar_tracing 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(ekf 
  src/ekf_test.cpp
  src/ekf.cpp
)
target_link_libraries(ekf 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)