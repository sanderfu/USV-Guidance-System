cmake_minimum_required(VERSION 3.0.2)
project(usv_colav)

find_package(catkin_simple REQUIRED)
catkin_python_setup()
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

find_library(gdal_LIBRARIES
  gdal
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/sb_mpc.cpp
)

#add_library(graph_builder src/graph_builder.cpp)
#target_compile_options(graph_builder PRIVATE ${OpenMP_FLAGS})
#add_dependencies(graph_builder ${catkin_EXPORTED_TARGETS})
#target_link_libraries(graph_builder ${catkin_LIBRARIES} ${OpenMP_LIBS})
#install(TARGETS graph_builder
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

############
# BINARIES #
############




##########
# EXPORT #
##########
cs_install()
cs_export()

## Install python nodes
#set(py_nodes
#  static_mission_planner_node
#)

foreach(py_node IN LISTS py_nodes)
  catkin_install_python(PROGRAMS scripts/${py_node}.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()