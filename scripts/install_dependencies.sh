#!/bin/bash

function clone_dependencies() {
    cd /home/sanderfu/catkin_ws/src || exit 1
    git clone git@github.com:catkin/catkin_simple.git
    git clone git@github.com:ethz-asl/geodetic_utils.git
    git clone git@github.com:sanderfu/rviz_polygon_filled.git
}

function install_plotjuggler {
    sudo apt install ros-noetic-plotjuggler-ros
}

function install_geographic_lib{
    cd /home/sanderfu || exit 1
    git clone git@github.com:geographiclib/geographiclib.git
    cd geographiclib || exit 1
    mkdir BUILD
    cd BUILD
    cmake ..
    make
    make install

}

clone_dependencies
install_plotjuggler
install_geographic_lib

