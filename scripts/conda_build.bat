REM Make sure to run this in the visual studio shell
REM (start menu -> Visual Studio 2019 -> x64 Native Tools Command Prompt for VS 2019)

REM Also, make sure only the base conda environment is activated
REM C:\Users\you\Miniconda3\Scripts\activate.bat

cd conda
call conda install -q -y anaconda-client conda-build
conda-build -c conda-forge .
cd ..
