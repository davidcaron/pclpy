#! /usr/bin/env bash

set -e

BASE=$(pwd)

cd generators || exit

PCL_REPO_PATH="${BASE}"/pcl-pcl-1.9.1 PYTHONPATH="${BASE}" python generate_yaml_point_types.py
PYTHONPATH="${BASE}" python generate_pybind11_bindings.py
