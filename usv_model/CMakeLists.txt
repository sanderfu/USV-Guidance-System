cmake_minimum_required(VERSION 3.0.2)
project(usv_model)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/model_library.cpp
)

############
# BINARIES #
############

##########
# EXPORT #
##########
cs_install()
cs_export()