#! /usr/bin/env bash

version="1.9.1"

python -m pip install https://github.com/davidcaron/CppHeaderParser/archive/master.zip

if [[ ! -d "pcl-pcl-$version" ]]; then
    scripts/download_pcl.sh
fi

if [[ ! -d "pclpy/src/generated_modules" ]]; then
    scripts/generate_points_and_bindings.sh
fi

python setup.py build

python -m pip install --no-deps .
