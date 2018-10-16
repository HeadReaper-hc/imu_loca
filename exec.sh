#!/bin/bash

cd build
rm -rf ./*
cmake ..
make -j4
cd ../bin
sudo ./readNav
