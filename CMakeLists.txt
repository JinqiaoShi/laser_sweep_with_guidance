cmake_minimum_required(VERSION 2.8.3)
project(laser_sweep_with_guidance)
set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  dynamixel_controllers
  laser_geometry
  sensor_msgs
  pcl_ros
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


catkin_package(

  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

link_directories(lib/x64)




include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
  include
)

link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

add_executable(sweep_node src/sweep_node.cpp)
add_executable(scan2pcl src/scan2pcl.cpp)
add_executable(calibration_node src/calibration_node.cpp)
add_executable(scanRegistration src/scanRegistration.cpp)
add_executable(laserOdometry src/laserOdometry.cpp)
add_executable(laserMapping src/laserMapping.cpp)
add_executable(transformMaintenance src/transformMaintenance.cpp)
add_executable(GuidanceNode src/GuidanceNode.cpp src/DJI_utility.cpp)

target_link_libraries(sweep_node ${catkin_LIBRARIES})
target_link_libraries(scan2pcl ${catkin_LIBRARIES})
target_link_libraries(calibration_node ${catkin_LIBRARIES})
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES})

target_link_libraries(GuidanceNode DJI_guidance usb-1.0 ${catkin_LIBRARIES} ${OpenCV_LIBS})

