cmake_minimum_required(VERSION 3.0.2)
project(thermal_range_calib)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(Threads)
find_package(Ceres REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(canny_test src/canny_test.cpp)
target_link_libraries(canny_test 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)

add_executable(laplace_test src/laplace_test.cpp)
target_link_libraries(laplace_test 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)

add_executable(laplace_test_non_max src/laplace_test_non_max.cpp)
target_link_libraries(laplace_test_non_max 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)

add_executable(counter_test src/counter_test.cpp)
target_link_libraries(counter_test 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)



add_executable(thermal_range_calib_ooad src/thermal_range_calib_ooad.cpp include/thermal_range_calib.hpp)
target_link_libraries(thermal_range_calib_ooad 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)

add_executable(thermal_range_calib_ooad_multi src/thermal_range_calib_ooad_multi.cpp include/thermal_range_calib.hpp)
target_link_libraries(thermal_range_calib_ooad_multi 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)

add_executable(bag_to_pcd src/bag_to_pcd.cpp)
target_link_libraries(bag_to_pcd 
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}
)



