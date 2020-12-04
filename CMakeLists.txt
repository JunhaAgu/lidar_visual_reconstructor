cmake_minimum_required(VERSION 2.8.3)
project(visual_reconstructor)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge

  std_msgs
  message_generation
  
  sensor_msgs
  
  pcl_conversions
  pcl_ros
  message_filters
  image_transport
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  profile3d.msg
)

## Generate added messages and services with any dependencies listed here.
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS message_runtime roscpp message_filters
)


set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

set(DCMAKE_build_type debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg -O2 -m64 -Ofast -flto -msse3 -march=native -mfpmath=sse")


add_executable (vis_recon src/main_vis_recon.cpp ${SRC})
target_link_libraries (vis_recon ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable (communicator src/main_communicator.cpp ${SRC})
target_link_libraries (communicator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

