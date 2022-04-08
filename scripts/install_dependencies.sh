#!/bin/bash

#Clonde dependencies
cd /home/sanderfu/catkin_ws/src || exit 1
git clone git@github.com:catkin/catkin_simple.git
git clone git@github.com:ethz-asl/geodetic_utils.git
git clone git@github.com:sanderfu/rviz_polygon_filled.git
git clone git@github.com:sanderfu/enc-extraction.git
git clone git@github.com:sanderfu/planner_commons.git



#Install GeographicLib
cd /home/sanderfu || exit 1
git clone git@github.com:geographiclib/geographiclib.git
cd geographiclib || exit 1
mkdir BUILD
cd BUILD
cmake ..
make
make install

#Install GDAL/OGR
