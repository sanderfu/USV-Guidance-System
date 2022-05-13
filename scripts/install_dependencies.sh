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

#Visualization dependenices
sudo apt install python3.8-tk
pip install pandas
pip install tqdm
pip install GDAL==3.0.4
pip install descartes
pip install shapely

#Install GDAL/OGR
