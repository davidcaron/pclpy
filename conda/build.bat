
python -m pip install https://github.com/davidcaron/CppHeaderParser/archive/master.zip

powershell -ExecutionPolicy ByPass -File scripts\download_pcl.ps1

call scripts\generate_points_and_bindings.bat

python -m pip install . --no-deps