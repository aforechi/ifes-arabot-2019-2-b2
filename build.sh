#!/usr/bin/env bash

rm -rf build/ && mkdir build && cd build
cmake ../AirSim/cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w"
make
