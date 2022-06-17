cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(tf_publish)

set(PACKAGE_DEPENDENCIES
  roscpp
  sensor_msgs
  std_srvs
  std_msgs
  eigen_conversions
)


find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
include_directories(${catkin_INCLUDE_DIRS}  )
add_executable(pcl_tf pcl_tf.cpp)
target_link_libraries(pcl_tf ${catkin_LIBRARIES})


