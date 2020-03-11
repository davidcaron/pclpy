#! /usr/bin/env bash

$PYTHON -m pip install https://github.com/davidcaron/CppHeaderParser/archive/master.zip

scripts/download_pcl.sh

if [[ ! -d "pclpy/src/generated_modules" ]]; then
    scripts/generate_points_and_bindings.sh
fi

$PYTHON -m pip install . --no-deps
