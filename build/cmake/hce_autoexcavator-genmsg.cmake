# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hce_autoexcavator: 0 messages, 4 services")

set(MSG_I_FLAGS "-Ihce_autoexcavator:/home/junhakim/hce_ws/src/lidar_visual_reconstructor/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hce_autoexcavator_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv" NAME_WE)
add_custom_target(_hce_autoexcavator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hce_autoexcavator" "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv" "std_msgs/Header"
)

get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv" NAME_WE)
add_custom_target(_hce_autoexcavator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hce_autoexcavator" "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv" "std_msgs/Header"
)

get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv" NAME_WE)
add_custom_target(_hce_autoexcavator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hce_autoexcavator" "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv" "sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv" NAME_WE)
add_custom_target(_hce_autoexcavator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hce_autoexcavator" "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_cpp(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_cpp(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_cpp(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hce_autoexcavator
)

### Generating Module File
_generate_module_cpp(hce_autoexcavator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hce_autoexcavator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hce_autoexcavator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hce_autoexcavator_generate_messages hce_autoexcavator_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_cpp _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_cpp _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_cpp _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_cpp _hce_autoexcavator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hce_autoexcavator_gencpp)
add_dependencies(hce_autoexcavator_gencpp hce_autoexcavator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hce_autoexcavator_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_eus(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_eus(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_eus(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hce_autoexcavator
)

### Generating Module File
_generate_module_eus(hce_autoexcavator
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hce_autoexcavator
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hce_autoexcavator_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hce_autoexcavator_generate_messages hce_autoexcavator_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_eus _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_eus _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_eus _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_eus _hce_autoexcavator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hce_autoexcavator_geneus)
add_dependencies(hce_autoexcavator_geneus hce_autoexcavator_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hce_autoexcavator_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_lisp(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_lisp(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_lisp(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hce_autoexcavator
)

### Generating Module File
_generate_module_lisp(hce_autoexcavator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hce_autoexcavator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hce_autoexcavator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hce_autoexcavator_generate_messages hce_autoexcavator_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_lisp _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_lisp _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_lisp _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_lisp _hce_autoexcavator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hce_autoexcavator_genlisp)
add_dependencies(hce_autoexcavator_genlisp hce_autoexcavator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hce_autoexcavator_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_nodejs(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_nodejs(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_nodejs(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hce_autoexcavator
)

### Generating Module File
_generate_module_nodejs(hce_autoexcavator
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hce_autoexcavator
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hce_autoexcavator_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hce_autoexcavator_generate_messages hce_autoexcavator_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_nodejs _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_nodejs _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_nodejs _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_nodejs _hce_autoexcavator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hce_autoexcavator_gennodejs)
add_dependencies(hce_autoexcavator_gennodejs hce_autoexcavator_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hce_autoexcavator_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_py(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_py(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hce_autoexcavator
)
_generate_srv_py(hce_autoexcavator
  "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hce_autoexcavator
)

### Generating Module File
_generate_module_py(hce_autoexcavator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hce_autoexcavator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hce_autoexcavator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hce_autoexcavator_generate_messages hce_autoexcavator_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePointsStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_py _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/profilePolynomialStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_py _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/lidarImageDataStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_py _hce_autoexcavator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/junhakim/hce_ws/src/lidar_visual_reconstructor/srv/relativeLidarPoseStamped.srv" NAME_WE)
add_dependencies(hce_autoexcavator_generate_messages_py _hce_autoexcavator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hce_autoexcavator_genpy)
add_dependencies(hce_autoexcavator_genpy hce_autoexcavator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hce_autoexcavator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hce_autoexcavator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hce_autoexcavator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hce_autoexcavator_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(hce_autoexcavator_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hce_autoexcavator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hce_autoexcavator
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hce_autoexcavator_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(hce_autoexcavator_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hce_autoexcavator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hce_autoexcavator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hce_autoexcavator_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(hce_autoexcavator_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hce_autoexcavator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hce_autoexcavator
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hce_autoexcavator_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(hce_autoexcavator_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hce_autoexcavator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hce_autoexcavator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hce_autoexcavator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hce_autoexcavator_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(hce_autoexcavator_generate_messages_py sensor_msgs_generate_messages_py)
endif()
