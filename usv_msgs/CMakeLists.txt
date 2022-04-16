cmake_minimum_required(VERSION 3.0.2)
project(usv_msgs)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}
#  src/sim_vessel.cpp
# src/sim_land.cpp
#  src/sim_tracker.cpp
#  src/monte_carlo_supervisor.cpp
#)

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


##########
# EXPORT #
##########
cs_install()
cs_export()
