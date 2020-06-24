# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "drone_code: 1 messages, 0 services")

set(MSG_I_FLAGS "-Idrone_code:/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(drone_code_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg" NAME_WE)
add_custom_target(_drone_code_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_code" "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/PoseStamped:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(drone_code
  "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_code
)

### Generating Services

### Generating Module File
_generate_module_cpp(drone_code
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_code
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(drone_code_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(drone_code_generate_messages drone_code_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg" NAME_WE)
add_dependencies(drone_code_generate_messages_cpp _drone_code_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_code_gencpp)
add_dependencies(drone_code_gencpp drone_code_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_code_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(drone_code
  "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_code
)

### Generating Services

### Generating Module File
_generate_module_eus(drone_code
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_code
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(drone_code_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(drone_code_generate_messages drone_code_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg" NAME_WE)
add_dependencies(drone_code_generate_messages_eus _drone_code_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_code_geneus)
add_dependencies(drone_code_geneus drone_code_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_code_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(drone_code
  "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_code
)

### Generating Services

### Generating Module File
_generate_module_lisp(drone_code
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_code
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(drone_code_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(drone_code_generate_messages drone_code_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg" NAME_WE)
add_dependencies(drone_code_generate_messages_lisp _drone_code_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_code_genlisp)
add_dependencies(drone_code_genlisp drone_code_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_code_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(drone_code
  "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_code
)

### Generating Services

### Generating Module File
_generate_module_nodejs(drone_code
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_code
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(drone_code_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(drone_code_generate_messages drone_code_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg" NAME_WE)
add_dependencies(drone_code_generate_messages_nodejs _drone_code_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_code_gennodejs)
add_dependencies(drone_code_gennodejs drone_code_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_code_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(drone_code
  "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_code
)

### Generating Services

### Generating Module File
_generate_module_py(drone_code
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_code
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(drone_code_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(drone_code_generate_messages drone_code_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/mnt/hgfs/Thesis2019_code/workspace/src/drone_code/msg/coord.msg" NAME_WE)
add_dependencies(drone_code_generate_messages_py _drone_code_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_code_genpy)
add_dependencies(drone_code_genpy drone_code_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_code_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_code)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_code
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(drone_code_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(drone_code_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_code)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_code
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(drone_code_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(drone_code_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_code)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_code
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(drone_code_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(drone_code_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_code)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_code
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(drone_code_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(drone_code_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_code)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_code\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_code
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(drone_code_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(drone_code_generate_messages_py geometry_msgs_generate_messages_py)
endif()
