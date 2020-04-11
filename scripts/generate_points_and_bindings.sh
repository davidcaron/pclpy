#! /usr/bin/env bash

set -e

BASE=$(pwd)

cd generators || exit

export PCL_REPO_PATH="${BASE}"/pcl-pcl-1.9.1
export PYTHONPATH="${BASE}"
# set to 'all' to generate all point types (slower to compile)
export POINT_GROUPS="$1"

${PYTHON:-python} generate_yaml_point_types.py
${PYTHON:-python} generate_pybind11_bindings.py
