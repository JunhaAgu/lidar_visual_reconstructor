cmake_minimum_required(VERSION 2.8.3)
project(hce_autoexcavator)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge

  message_generation

  std_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  message_filters
  image_transport
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ControlInputsStamped.msg
)

add_service_files(
  FILES
  ProfilePolynomialStamped.srv
  ProfilePointsStamped.srv
  LidarImageDataStamped.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS message_runtime roscpp message_filters
)

set(SRC ${SRC} 
  src/hcegcs.cpp
  src/lidar_visual_reconstructor.cpp
  src/feature_tracker.cpp)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

set(DCMAKE_build_type debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg -O2 -m64 -Ofast -flto -msse3 -march=native -mfpmath=sse")


# executables
add_executable (reconstruction_node src/main_reconstruction.cpp ${SRC})
target_link_libraries (reconstruction_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable (gcs_node src/main_gcs.cpp ${SRC})
target_link_libraries (gcs_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})