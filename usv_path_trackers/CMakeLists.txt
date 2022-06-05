cmake_minimum_required(VERSION 3.0.2)
project(usv_path_trackers)

find_package(catkin_simple REQUIRED)
catkin_python_setup()
catkin_simple(ALL_DEPS_REQUIRED)

##########
# EXPORT #
##########
cs_install()
cs_export()

## Install python nodes
set(py_nodes
  los_controller_node_usv
  los_controller_node_obstacle
)

foreach(py_node IN LISTS py_nodes)
  catkin_install_python(PROGRAMS scripts/${py_node}.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()