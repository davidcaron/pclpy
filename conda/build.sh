#! /usr/bin/env bash

version="1.9.1"

$PYTHON -m pip install https://github.com/davidcaron/CppHeaderParser/archive/master.zip

if [[ ! -d "pcl-pcl-$version" ]]; then
    scripts/download_pcl.sh
fi

if [[ ! -d "pclpy/src/generated_modules" ]]; then
    scripts/generate_points_and_bindings.sh
fi

$PYTHON setup.py build

$PYTHON -m pip install . --no-deps
