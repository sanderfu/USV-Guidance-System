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
  src/skeleton_generator.cpp
  src/jc_voronoi.cpp
  src/geotf_server.cpp
  src/map_service.cpp
  src/map_preprocessor.cpp
  src/geotf_sync.cpp
)
target_link_libraries(${PROJECT_NAME} region ${GeographicLib_LIBRARIES})

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

cs_add_executable(geotf_server_node
  src/geotf_server_node.cpp
)
target_link_libraries(geotf_server_node ${PROJECT_NAME} ${gdal_LIBRARIES})

cs_add_executable(map_preprocessor_node
  src/map_preprocessor_node.cpp
)
target_link_libraries(map_preprocessor_node ${PROJECT_NAME} ${gdal_LIBRARIES} ${GeographicLib_LIBRARIES})

cs_add_executable(test_map_preprocessor
  src/test_map_preprocessor.cpp
)
target_link_libraries(test_map_preprocessor ${PROJECT_NAME} ${GeographicLib_LIBRARIES} ${gdal_LIBRARIES})

cs_add_executable(test_artificial_field
  src/test_artificial_field.cpp
)
target_link_libraries(test_artificial_field ${PROJECT_NAME} ${GeographicLib_LIBRARIES} ${gdal_LIBRARIES})


##########
# EXPORT #
##########
cs_install()
cs_export()