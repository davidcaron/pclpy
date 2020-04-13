@ECHO OFF
SET "base=%~dp0..\"

cd generators

set PCL_REPO_PATH=%base%pcl-pcl-1.9.1
set PYTHONPATH=%base%
REM set to 'all' to generate all point types (slower to compile)
set POINT_GROUPS=%1

IF "%PYTHON%"=="" set PYTHON=python

@ECHO ON
%PYTHON% generate_yaml_point_types.py || exit /b
%PYTHON% generate_pybind11_bindings.py || exit /b

@ECHO OFF
cd ..
