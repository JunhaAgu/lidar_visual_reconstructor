cmake_minimum_required(VERSION 2.8.3)
project(hce_autoexcavator)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -pg -O3 -m64 -Ofast -flto -msse -msse2 -msse3 -mavx2 -march=native -mfpmath=sse")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pg -O3 -m64 -Ofast -flto -msse -msse2 -msse3 -march=native -mfpmath=sse")

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen3 REQUIRED)
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

  hce_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   controlInputsStamped.msg
#   packetsFromExcavator.msg
#   packetsToExcavator.msg
# )

# add_service_files(
#   FILES
#   profilePolynomialStamped.srv
#   profilePointsStamped.srv
#   lidarImageDataStamped.srv
#   relativeLidarPoseStamped.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
# )

catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS message_runtime roscpp message_filters
)

set(SRC ${SRC} 
  src/hcegcs.cpp
  src/lidar_visual_reconstructor.cpp
  
  src/epipolar_klt.cpp
  src/frame.cpp
  src/camera.cpp

  src/util/image_procesing.cpp
  src/util/timer.cpp
  src/util/sophus_lie.cpp

  src/cdt/adjacent_map.cpp
  src/cdt/constrained_delaunay.cpp

  src/can/can_communicator.cpp
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

# executables
add_executable (reconstruction_node src/main_reconstruction.cpp ${SRC})
target_link_libraries (reconstruction_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable (gcs_node src/main_gcs.cpp ${SRC})
target_link_libraries (gcs_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(gcs_node lidarImageDataStamped_gencpp)
