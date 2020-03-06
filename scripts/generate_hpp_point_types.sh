#! /usr/bin/env bash

BASE=$(pwd)

cd generators || exit

PCL_REPO_PATH="${BASE}"/pcl-pcl-1.9.1 PYTHONPATH="${BASE}" python generate_point_types.py
