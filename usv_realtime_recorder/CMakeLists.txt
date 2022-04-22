cmake_minimum_required(VERSION 3.0.2)
project(usv_realtime_recorder)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/recorder.cpp
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

cs_add_executable(recorder_node
  src/recorder_node.cpp
)
target_link_libraries(recorder_node ${PROJECT_NAME})



##########
# EXPORT #
##########
cs_install()
cs_export()