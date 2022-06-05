cmake_minimum_required(VERSION 3.0.2)
project(usv_mission_planner)
add_compile_options(-std=c++17)
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

find_package (GeographicLib REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/astar.cpp
  src/hybrid_astar.cpp
  src/mission_planner.cpp
  src/dubins.cpp
)


############
# BINARIES #
############
#cs_add_executable(test_quadtree
#  src/test_quadtree.cpp
#)
#target_link_libraries(test_quadtree ${PROJECT_NAME} ${GeographicLib_LIBRARIES} ${gdal_LIBRARIES})

cs_add_executable(mission_planner_node
  src/mission_planner_node.cpp
)
target_link_libraries(mission_planner_node ${PROJECT_NAME} ${GeographicLib_LIBRARIES} ${gdal_LIBRARIES})

cs_add_executable(mission_planner_client_node
  src/mission_planner_client_node.cpp
)
target_link_libraries(mission_planner_client_node ${PROJECT_NAME} ${GeographicLib_LIBRARIES} ${gdal_LIBRARIES})


##########
# EXPORT #
##########
cs_install()
cs_export()