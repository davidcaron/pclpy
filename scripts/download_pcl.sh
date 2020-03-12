#! /usr/bin/env bash

version="1.9.1"

if [[ ! -d "pcl-pcl-$version" ]]; then
    wget --no-verbose -c https://github.com/PointCloudLibrary/pcl/archive/pcl-"$version".tar.gz -O - | tar -xz
fi