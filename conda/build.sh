#! /usr/bin/env bash

version="1.9.1"


if [[ ! -d "pcl-pcl-$version" ]]; then
    scripts/download_pcl.sh
fi

if [[ ! -d "pclpy/src/generated_modules" ]]; then
    python -m pip install https://github.com/davidcaron/CppHeaderParser/archive/master.zip
    scripts/generate_points_and_bindings.sh
    python -m pip uninstall CppHeaderParser
fi


python -m pip install . --no-deps -vv
