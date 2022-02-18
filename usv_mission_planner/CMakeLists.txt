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
add_library(region src/region.cpp)
target_compile_options(region PRIVATE ${OpenMP_FLAGS})
add_dependencies(region ${catkin_EXPORTED_TARGETS})
target_link_libraries(region ${catkin_LIBRARIES} ${OpenMP_LIBS})
install(TARGETS region
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cs_add_library(${PROJECT_NAME}
  src/quadtree.cpp
  src/astar.cpp
)
target_link_libraries(${PROJECT_NAME} region)

############
# BINARIES #
############
cs_add_executable(test_quadtree
  src/test_quadtree.cpp
)
target_link_libraries(test_quadtree ${PROJECT_NAME} ${GeographicLib_LIBRARIES} ${gdal_LIBRARIES})



##########
# EXPORT #
##########
cs_install()
cs_export()

## Install python nodes
set(py_nodes
  static_mission_planner_node
)

foreach(py_node IN LISTS py_nodes)
  catkin_install_python(PROGRAMS scripts/${py_node}.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()