cmake_minimum_required(VERSION 3.0.2)
project(usv_map)

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
  src/geotf_ros.cpp
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

#cs_add_executable(dynamic_planner_node
#  src/dynamic_planner_node.cpp
#)
#target_link_libraries(dynamic_planner_node ${PROJECT_NAME} graph_builder)

#cs_add_executable(sim_land_node
#  src/sim_land_node.cpp
#)
#target_link_libraries(sim_land_node ${PROJECT_NAME} ${gdal_LIBRARIES})

cs_add_executable(geotf_ros_node
  src/geotf_ros_node.cpp
)
target_link_libraries(geotf_ros_node ${PROJECT_NAME} ${gdal_LIBRARIES})

cs_add_executable(test_services
  src/test_services.cpp
)
target_link_libraries(test_services ${PROJECT_NAME})



##########
# EXPORT #
##########
cs_install()
cs_export()

#Install python nodes
set(py_nodes
  test_client
)

foreach(py_node IN LISTS py_nodes)
  catkin_install_python(PROGRAMS scripts/${py_node}.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()