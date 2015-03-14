# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ardrone_application_node: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iardrone_application_node:/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ardrone_application_node_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg" NAME_WE)
add_custom_target(_ardrone_application_node_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_application_node" "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg" ""
)

get_filename_component(_filename "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command.msg" NAME_WE)
add_custom_target(_ardrone_application_node_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ardrone_application_node" "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command.msg" "ardrone_application_node/serialized_ardrone_command_part"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ardrone_application_node
  "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_application_node
)
_generate_msg_cpp(ardrone_application_node
  "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command.msg"
  "${MSG_I_FLAGS}"
  "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_application_node
)

### Generating Services

### Generating Module File
_generate_module_cpp(ardrone_application_node
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_application_node
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ardrone_application_node_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ardrone_application_node_generate_messages ardrone_application_node_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg" NAME_WE)
add_dependencies(ardrone_application_node_generate_messages_cpp _ardrone_application_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command.msg" NAME_WE)
add_dependencies(ardrone_application_node_generate_messages_cpp _ardrone_application_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardrone_application_node_gencpp)
add_dependencies(ardrone_application_node_gencpp ardrone_application_node_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardrone_application_node_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ardrone_application_node
  "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_application_node
)
_generate_msg_lisp(ardrone_application_node
  "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command.msg"
  "${MSG_I_FLAGS}"
  "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_application_node
)

### Generating Services

### Generating Module File
_generate_module_lisp(ardrone_application_node
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_application_node
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ardrone_application_node_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ardrone_application_node_generate_messages ardrone_application_node_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg" NAME_WE)
add_dependencies(ardrone_application_node_generate_messages_lisp _ardrone_application_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command.msg" NAME_WE)
add_dependencies(ardrone_application_node_generate_messages_lisp _ardrone_application_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardrone_application_node_genlisp)
add_dependencies(ardrone_application_node_genlisp ardrone_application_node_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardrone_application_node_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ardrone_application_node
  "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_application_node
)
_generate_msg_py(ardrone_application_node
  "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command.msg"
  "${MSG_I_FLAGS}"
  "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_application_node
)

### Generating Services

### Generating Module File
_generate_module_py(ardrone_application_node
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_application_node
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ardrone_application_node_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ardrone_application_node_generate_messages ardrone_application_node_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command_part.msg" NAME_WE)
add_dependencies(ardrone_application_node_generate_messages_py _ardrone_application_node_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hewhosurvives/c++/randomProjects/ros/catkin_ws/src/ardrone_application_node/msg/serialized_ardrone_command.msg" NAME_WE)
add_dependencies(ardrone_application_node_generate_messages_py _ardrone_application_node_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ardrone_application_node_genpy)
add_dependencies(ardrone_application_node_genpy ardrone_application_node_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ardrone_application_node_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_application_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ardrone_application_node
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(ardrone_application_node_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(ardrone_application_node_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_application_node)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ardrone_application_node
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(ardrone_application_node_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(ardrone_application_node_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_application_node)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_application_node\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ardrone_application_node
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(ardrone_application_node_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(ardrone_application_node_generate_messages_py std_msgs_generate_messages_py)
