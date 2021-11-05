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


find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


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
)

add_executable(radar_odometry 
  src/radar_odometry.cpp
)
target_link_libraries(radar_odometry 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(odometry_demo 
  src/odometry_demo.cpp
)
target_link_libraries(odometry_demo 
  ${catkin_LIBRARIES}
)