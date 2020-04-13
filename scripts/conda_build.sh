#! /usr/bin/env bash

set -e

conda install -q -y anaconda-client conda-build
cd conda
conda-build -c conda-forge .
cd ..
